#include "ecs/world_system_scheduler.h"

#include "core/cpu_profiling.h"
#include "core/log.h"

#include "ecs/observable_member.h"
#include "ecs/reflection.h"
#include "ecs/system.h"
#include "ecs/update_groups.h"
#include "ecs/world.h"

#include <rttr/policy.h>
#include <rttr/registration>

#include <algorithm>
#include <cstdio>
#include <exception>
#include <set>
#include <unordered_set>
#include <utility>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <pthread.h>
#include <sched.h>
#endif

namespace era_engine
{
	namespace
	{
		std::string to_std(const rttr::string_view& view)
		{
			return std::string(view.data(), view.size());
		}

		std::string read_string_metadata(const rttr::variant& meta, const std::string& fallback, const std::string& context)
		{
			if (!meta.is_valid())
			{
				return fallback;
			}

			if (meta.is_type<std::string>())
			{
				return meta.get_value<std::string>();
			}

			if (meta.can_convert<std::string>())
			{
				return meta.to_string();
			}

			LOG_WARNING(std::format("{}: metadata is not convertible to a string, using  {}.", context, fallback).c_str());
			return fallback;
		}

		std::vector<std::string> read_string_list_metadata(const rttr::variant& meta, const std::string& context)
		{
			if (!meta.is_valid())
			{
				return std::vector<std::string>{};
			}

			if (meta.is_type<std::vector<std::string>>())
			{
				return meta.get_value<std::vector<std::string>>();
			}

			if (meta.is_type<std::string>())
			{
				return std::vector<std::string>{ meta.get_value<std::string>() };
			}
			LOG_WARNING(std::format("{}: metadata is neither a string nor a vector<string>, ignored.", context).c_str());
			return std::vector<std::string>{};
		}

		// Only the fixed *timer* thread gets a bump.
		void nudge_thread_priority(std::thread& thread)
		{
			if (!thread.joinable())
			{
				return;
			}

#ifdef _WIN32
			::SetThreadPriority(thread.native_handle(), THREAD_PRIORITY_ABOVE_NORMAL);
#else
			sched_param params{};
			int policy = 0;
			pthread_t handle = thread.native_handle();
			if (pthread_getschedparam(handle, &policy, &params) == 0)
			{
				// Deliberately stay inside the current (normally SCHED_OTHER) policy.
				const int max_priority = sched_get_priority_max(policy);
				params.sched_priority = std::min(params.sched_priority + 1, max_priority);
				pthread_setschedparam(handle, policy, &params);
			}
#endif
		}

		void sleep_until_precise(std::chrono::steady_clock::time_point target,
			std::chrono::steady_clock::duration spin_margin,
			const std::atomic<bool>& running)
		{
			using clock = std::chrono::steady_clock;

			const clock::time_point now = clock::now();
			if (target <= now)
			{
				return;
			}

			const clock::duration remaining = target - now;
			if (remaining > spin_margin)
			{
				std::this_thread::sleep_for(remaining - spin_margin);
			}

			while (running.load(std::memory_order_relaxed) && clock::now() < target)
			{
				std::this_thread::yield();
			}
		}

		size_t clamp_thread_count(size_t requested)
		{
			const size_t hardware = static_cast<size_t>(std::max(1u, std::thread::hardware_concurrency()));

			// The calling thread participates in the pool, so leave a core for it.
			const size_t budget = hardware > 1 ? hardware - 1 : 1;

			if (requested == 0)
			{
				return budget;
			}

			if (requested > budget)
			{
				LOG_WARNING(std::format("Worker thread count {} exceeds the core budget, clamped to {}.", requested, budget).c_str());
				return budget;
			}

			return requested;
		}
	}

	WorldSystemScheduler::WorldSystemScheduler(World* _world, size_t normal_threads, size_t fixed_threads)
		: world(_world)
	{
		ASSERT(world != nullptr);

		update_types::register_default_order();

		set_fixed_update_rate(fixed_update_rate.load(std::memory_order_relaxed));

		const size_t thread_count = clamp_thread_count(normal_threads + fixed_threads);

		running.store(true, std::memory_order_relaxed);

		workers.reserve(thread_count);
		for (size_t i = 0; i < thread_count; ++i)
		{
			workers.emplace_back(&WorldSystemScheduler::worker_loop, this);
		}
	}

	WorldSystemScheduler::~WorldSystemScheduler()
	{
		stop();

		{
			std::lock_guard<std::mutex> lock(schedule_mutex);
			normal_schedule.reset();
			fixed_schedule.reset();
		}

		tasks.clear();
		fixed_tasks.clear();

		// rttr hands out raw pointers.
		for (System* system : systems)
		{
			delete system;
		}
		systems.clear();
		system_types.clear();
	}

	void WorldSystemScheduler::stop()
	{
		running.store(false, std::memory_order_relaxed);

		{
			// Taking the lock guarantees no thread sits between its predicate check and wait().
			std::lock_guard<std::mutex> lock(work_mutex);
		}

		work_available.notify_all();

		if (fixed_timer_thread.joinable())
		{
			fixed_timer_thread.join();
		}

		for (std::thread& thread : workers)
		{
			if (thread.joinable())
			{
				thread.join();
			}
		}
		workers.clear();

		std::lock_guard<std::mutex> lock(work_mutex);
		for (size_t kind = 0; kind < QUEUE_COUNT; ++kind)
		{
			queues[kind].clear();
		}
	}

	bool WorldSystemScheduler::is_running() const
	{
		return running.load(std::memory_order_relaxed);
	}

	void WorldSystemScheduler::set_fixed_update_rate(double rate_hz)
	{
		if (rate_hz <= 0.0)
		{
			LOG_ERROR("set_fixed_update_rate: non-positive rate ignored.");
			return;
		}

		fixed_update_rate.store(rate_hz, std::memory_order_relaxed);

		if (world != nullptr)
		{
			world->fixed_update_dt = 1.0f / static_cast<float>(rate_hz);
		}
	}

	double WorldSystemScheduler::get_fixed_update_rate() const
	{
		return fixed_update_rate.load(std::memory_order_relaxed);
	}

	void WorldSystemScheduler::set_fixed_step_mode(FixedStepMode mode)
	{
		fixed_step_mode.store(mode, std::memory_order_relaxed);
		pump_clock_valid = false;
	}

	FixedStepMode WorldSystemScheduler::get_fixed_step_mode() const
	{
		return fixed_step_mode.load(std::memory_order_relaxed);
	}

	void WorldSystemScheduler::set_max_fixed_steps_per_wakeup(uint32 steps)
	{
		max_fixed_steps_per_wakeup.store(std::max<uint32>(1u, steps), std::memory_order_relaxed);
	}

	void WorldSystemScheduler::set_group_overlap(const std::string& group_name, bool may_overlap_previous)
	{
		const bool changed = may_overlap_previous ? overlapping_groups.insert(group_name).second
			: overlapping_groups.erase(group_name) != 0;
		if (changed)
		{
			refresh_graph();
		}
	}

	void WorldSystemScheduler::set_overlapping_groups(const std::vector<std::string>& group_names)
	{
		overlapping_groups.clear();
		overlapping_groups.insert(group_names.begin(), group_names.end());

		refresh_graph();
	}

	bool WorldSystemScheduler::get_group_overlap(const std::string& group_name) const
	{
		return overlapping_groups.find(group_name) != overlapping_groups.end();
	}

	std::chrono::steady_clock::duration WorldSystemScheduler::fixed_interval() const
	{
		const double seconds = 1.0 / fixed_update_rate.load(std::memory_order_relaxed);
		return std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(seconds));
	}

	void WorldSystemScheduler::initialize_systems(const rttr::array_range<rttr::type>& types)
	{
		using namespace rttr;

		const type base_type = type::get<System>();
		const type world_ptr_type = type::get<World*>();

		for (const type& type_instance : types)
		{
			const type wrapped_type = type_instance.get_raw_type();
			if (!base_type.is_base_of(type_instance) || wrapped_type == base_type || !type_instance.is_class())
			{
				continue;
			}

			if (!system_types.insert(wrapped_type).second)
			{
				continue;
			}

			const constructor ctor = type_instance.get_constructor({ world_ptr_type });
			if (!ctor.is_valid())
			{
				system_types.erase(wrapped_type);
				LOG_ERROR(std::format("{}: no (World*) constructor registered, system skipped.", to_std(type_instance.get_name())).c_str());
				continue;
			}

			System* system = nullptr;
			variant created = type_instance.create({ world });
			if (created.is_valid() && created.can_convert<System*>())
			{
				system = created.convert<System*>();
			}

			if (system == nullptr)
			{
				system_types.erase(wrapped_type);
				LOG_ERROR(std::format("{}: construction failed, system skipped.", to_std(type_instance.get_name())).c_str());
				continue;
			}

			systems.push_back(system);

			const std::string system_tag = read_string_metadata(ctor.get_metadata("Tag"), "base",
				to_std(type_instance.get_name()) + " Tag");

			for (const method& system_method : type_instance.get_methods())
			{
				const variant meta = system_method.get_metadata("update_group");
				if (!meta.is_valid())
				{
					continue;
				}

				const array_range<parameter_info> params = system_method.get_parameter_infos();
				if (params.size() != 1)
				{
					LOG_ERROR(std::format("{}::{}: update methods must take exactly one (float) parameter, skipped.", to_std(type_instance.get_name()), to_std(system_method.get_name())).c_str());
					ASSERT(params.size() == 1);
					continue;
				}

				if (!meta.is_type<UpdateGroup>())
				{
					LOG_ERROR(std::format("{}::{}: update_group metadata has an unexpected type, skipped.", to_std(type_instance.get_name()), to_std(system_method.get_name())).c_str());
					continue;
				}

				const UpdateGroup group = meta.get_value<UpdateGroup>();

				const std::string method_context =
					to_std(type_instance.get_name()) + "::" + to_std(system_method.get_name());

				const std::vector<std::string> dependencies =
					read_string_list_metadata(system_method.get_metadata("After"), method_context + " After");

				const std::vector<std::string> dependents =
					read_string_list_metadata(system_method.get_metadata("Before"), method_context + " Before");

				add_task(make_ref<Task>(system, system_method, std::string(group.name), system_tag, dependencies, dependents),
					group.update_type);
			}
		}

		inited = true;
	}

	void WorldSystemScheduler::initialize_all_systems()
	{
		if (systems_initialized)
		{
			LOG_WARNING("initialize_all_systems() called more than once, ignored.");
			return;
		}
		systems_initialized = true;

		refresh_graph();

		for (System* system : systems)
		{
			system->init();
		}

		refresh_graph();

		next_fixed_update = std::chrono::steady_clock::now() + fixed_interval();
		fixed_timer_thread = std::thread(&WorldSystemScheduler::fixed_timer_loop, this);
		nudge_thread_priority(fixed_timer_thread);
	}

	void WorldSystemScheduler::refresh_graph()
	{
		if (!inited)
		{
			return;
		}

		UpdateScheduleRef new_normal = build_schedule(UpdateType::NORMAL);
		UpdateScheduleRef new_fixed = build_schedule(UpdateType::FIXED);

		std::lock_guard<std::mutex> lock(schedule_mutex);
		normal_schedule = std::move(new_normal);
		fixed_schedule = std::move(new_fixed);
	}

	UpdateScheduleRef WorldSystemScheduler::get_schedule(UpdateType type) const
	{
		std::lock_guard<std::mutex> lock(schedule_mutex);
		return type == UpdateType::NORMAL ? normal_schedule : fixed_schedule;
	}

	void WorldSystemScheduler::update_normal(float dt)
	{
		ZoneScopedN("WorldSystemScheduler::update_normal");

		if (!running.load(std::memory_order_relaxed))
		{
			return;
		}

		if (fixed_step_mode.load(std::memory_order_relaxed) == FixedStepMode::MAIN_THREAD_PUMP)
		{
			pump_fixed_steps();
		}

		run_schedule(get_schedule(UpdateType::NORMAL), dt, QUEUE_NORMAL);
	}

	void WorldSystemScheduler::update_fixed(float dt)
	{
		ZoneScopedN("WorldSystemScheduler::update_fixed");

		if (!running.load(std::memory_order_relaxed))
		{
			return;
		}

		std::lock_guard<std::mutex> step_lock(fixed_step_mutex);

		const std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

		run_schedule(get_schedule(UpdateType::FIXED), dt, QUEUE_FIXED);

		const double step_ms =
			std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count();

		stat_last_fixed_step_ms.store(step_ms, std::memory_order_relaxed);
		if (step_ms > stat_max_fixed_step_ms.load(std::memory_order_relaxed))
		{
			stat_max_fixed_step_ms.store(step_ms, std::memory_order_relaxed);
		}
		stat_fixed_steps.fetch_add(1, std::memory_order_relaxed);

		++world->fixed_frame_id;
	}

	void WorldSystemScheduler::run_schedule(const UpdateScheduleRef& schedule, float dt, size_t kind)
	{
		if (!schedule)
		{
			return;
		}

		for (const StageSchedule& stage : schedule->stages)
		{
			if (!running.load(std::memory_order_relaxed))
			{
				return;
			}

			run_stage(stage, dt, kind);
		}
	}

	void WorldSystemScheduler::run_stage(const StageSchedule& stage, float dt, size_t kind)
	{
		if (stage.nodes.empty())
		{
			return;
		}

		// A main-thread stage belongs entirely to the calling thread, in topological order.
		if (stage.serial || workers.empty() || stage.nodes.size() == 1)
		{
			for (const StageNode& node : stage.nodes)
			{
				node.task->invoke(dt);
			}
			return;
		}

		StageRun run;
		run.stage = &stage;
		run.dt = dt;
		run.total = static_cast<uint32>(stage.nodes.size());
		run.remaining.resize(stage.nodes.size());
		for (size_t i = 0; i < stage.nodes.size(); ++i)
		{
			run.remaining[i] = stage.nodes[i].predecessor_count;
		}

		{
			std::lock_guard<std::mutex> lock(work_mutex);
			for (uint32 root : stage.roots)
			{
				queues[kind].push_back(TaskItem{ &run, root });
			}
		}
		work_available.notify_all();

		// The driving thread is a worker too.
		while (true)
		{
			TaskItem item;
			{
				std::unique_lock<std::mutex> lock(work_mutex);
				work_available.wait(lock, [this, kind, &run] {
					return run.completed == run.total || !queues[kind].empty();
					});

				if (run.completed == run.total)
				{
					return;
				}

				if (!pop_from_locked(kind, item))
				{
					continue;
				}
			}

			execute_node(item, kind);
		}
	}

	bool WorldSystemScheduler::has_pending_locked() const
	{
		return !queues[QUEUE_NORMAL].empty() || !queues[QUEUE_FIXED].empty();
	}

	bool WorldSystemScheduler::pop_from_locked(size_t kind, TaskItem& out_item)
	{
		if (queues[kind].empty())
		{
			return false;
		}

		out_item = queues[kind].front();
		queues[kind].pop_front();

		return true;
	}

	bool WorldSystemScheduler::pop_any_locked(TaskItem& out_item, size_t& out_kind)
	{
		const size_t first = prefer_fixed ? static_cast<size_t>(QUEUE_FIXED) : static_cast<size_t>(QUEUE_NORMAL);
		const size_t second = prefer_fixed ? static_cast<size_t>(QUEUE_NORMAL) : static_cast<size_t>(QUEUE_FIXED);

		if (pop_from_locked(first, out_item))
		{
			out_kind = first;
			prefer_fixed = !prefer_fixed;
			return true;
		}

		if (pop_from_locked(second, out_item))
		{
			out_kind = second;
			prefer_fixed = !prefer_fixed;
			return true;
		}

		return false;
	}

	void WorldSystemScheduler::execute_node(const TaskItem& item, size_t kind)
	{
		StageRun& run = *item.run;
		const StageNode& node = run.stage->nodes[item.node];

		// StageRun::dt is written before the roots are published.
		node.task->invoke(run.dt);

		size_t released = 0;
		bool finished = false;
		{
			std::lock_guard<std::mutex> lock(work_mutex);

			for (uint32 successor : node.successors)
			{
				if (--run.remaining[successor] == 0)
				{
					queues[kind].push_back(TaskItem{ &run, successor });
					++released;
				}
			}

			++run.completed;
			finished = run.completed == run.total;
		}

		if (released != 0 || finished)
		{
			work_available.notify_all();
		}
	}

	void WorldSystemScheduler::worker_loop()
	{
		while (true)
		{
			TaskItem item;
			size_t kind = QUEUE_NORMAL;

			{
				std::unique_lock<std::mutex> lock(work_mutex);
				work_available.wait(lock, [this] {
					return !running.load(std::memory_order_relaxed) || has_pending_locked();
					});

				if (!pop_any_locked(item, kind))
				{
					if (!running.load(std::memory_order_relaxed))
					{
						break;
					}

					continue;
				}
			}

			execute_node(item, kind);
		}
	}

	void WorldSystemScheduler::fixed_timer_loop()
	{
		using clock = std::chrono::steady_clock;

		bool timer_active = false;

		while (running.load(std::memory_order_relaxed))
		{
			if (fixed_step_mode.load(std::memory_order_relaxed) != FixedStepMode::THREADED)
			{
				timer_active = false;
				std::this_thread::sleep_for(std::chrono::milliseconds(1));
				continue;
			}

			const clock::duration interval = fixed_interval();

			if (!timer_active)
			{
				timer_active = true;
				next_fixed_update = clock::now() + interval;
			}

			clock::time_point now = clock::now();
			if (now < next_fixed_update)
			{
				sleep_until_precise(next_fixed_update, fixed_spin_margin, running);
				continue;
			}

			const uint32 max_steps = std::max<uint32>(1u, max_fixed_steps_per_wakeup.load(std::memory_order_relaxed));
			uint32 steps = 0;

			while (running.load(std::memory_order_relaxed) && now >= next_fixed_update && steps < max_steps)
			{
				update_fixed(world->fixed_update_dt);

				next_fixed_update += interval;
				++steps;
				now = clock::now();
			}

			if (now >= next_fixed_update)
			{
				const clock::duration behind = now - next_fixed_update;
				const uint64 dropped = static_cast<uint64>(behind / interval) + 1ull;

				stat_dropped_fixed_steps.fetch_add(dropped, std::memory_order_relaxed);
				stat_fixed_overruns.fetch_add(1, std::memory_order_relaxed);

				next_fixed_update = now + interval;
				std::this_thread::yield();
			}
		}
	}

	uint32 WorldSystemScheduler::pump_fixed_steps()
	{
		using clock = std::chrono::steady_clock;

		const clock::duration interval = fixed_interval();
		const clock::time_point now = clock::now();

		if (!pump_clock_valid)
		{
			pump_clock_valid = true;
			next_pumped_fixed_update = now + interval;
			return 0;
		}

		const uint32 max_steps = std::max<uint32>(1u, max_fixed_steps_per_wakeup.load(std::memory_order_relaxed));
		uint32 steps = 0;

		while (next_pumped_fixed_update <= now && steps < max_steps)
		{
			update_fixed(world->fixed_update_dt);

			next_pumped_fixed_update += interval;
			++steps;
		}

		if (next_pumped_fixed_update <= now)
		{
			const clock::duration behind = now - next_pumped_fixed_update;
			const uint64 dropped = static_cast<uint64>(behind / interval) + 1ull;

			stat_dropped_fixed_steps.fetch_add(dropped, std::memory_order_relaxed);
			stat_fixed_overruns.fetch_add(1, std::memory_order_relaxed);

			next_pumped_fixed_update = now + interval;
		}

		return steps;
	}

	void WorldSystemScheduler::add_task(ref<Task> task, UpdateType type)
	{
		ASSERT(task != nullptr);

		std::unordered_map<std::string, ref<Task>>& target = type == UpdateType::NORMAL ? tasks : fixed_tasks;

		if (!target.emplace(task->name, task).second)
		{
			LOG_WARNING(std::format("Duplicate task {} ignored.", task->name).c_str());
		}
	}

	UpdateScheduleRef WorldSystemScheduler::build_schedule(UpdateType type) const
	{
		const std::unordered_map<std::string, ref<Task>>& source = type == UpdateType::NORMAL ? tasks : fixed_tasks;

		// Deterministic node order: unordered_map iteration order is unspecified.
		std::vector<std::string> names;
		names.reserve(source.size());
		for (const auto& entry : source)
		{
			names.push_back(entry.first);
		}
		std::sort(names.begin(), names.end());

		// Tag filtering is per task. Tasks of another tag are simply not part of the schedule and
		// dependencies pointing at them resolve to nothing.
		std::unordered_map<std::string, std::vector<std::string>> group_task_names;
		for (const std::string& task_name : names)
		{
			const ref<Task>& task = source.at(task_name);
			if (!world->has_tag(task->tag))
			{
				continue;
			}

			group_task_names[task->group].push_back(task_name);
		}

		std::set<std::pair<std::string, std::string>> edges;

		const auto declare_edge = [&](const std::string& from, const std::string& to, const char* relation) {
			const bool from_known = source.find(from) != source.end();
			const bool to_known = source.find(to) != source.end();

			if (!from_known || !to_known)
			{
				LOG_WARNING(std::format("Unknown task {} referenced by {} of {} dependency ignored.", from_known ? to : from, relation, from_known ? from : to).c_str());
				return;
			}

			if (from == to)
			{
				LOG_WARNING(std::format("Task {} depends on itself, dependency ignored.", from).c_str());
				return;
			}

			edges.emplace(from, to);
			};

		for (const std::string& task_name : names)
		{
			const ref<Task>& task = source.at(task_name);
			for (const std::string& dependency : task->dependencies)
			{
				declare_edge(dependency, task_name, "After");
			}
			for (const std::string& dependent : task->dependents)
			{
				declare_edge(task_name, dependent, "Before");
			}
		}

		struct StageBuilder
		{
			std::vector<std::string> group_names;
			std::vector<std::string> member_names;
			bool serial = false;
		};

		std::vector<StageBuilder> stages;
		std::unordered_set<std::string> scheduled_groups;

		for (const std::string& group_name : UpdatesHolder::update_order)
		{
			if (group_name.empty() || !scheduled_groups.insert(group_name).second)
			{
				continue;
			}

			const auto members_iter = group_task_names.find(group_name);
			if (members_iter == group_task_names.end() || members_iter->second.empty())
			{
				continue;
			}

			UpdateGroup* group = find_group(group_name);
			if (group == nullptr)
			{
				LOG_ERROR(std::format("Update group {} is in the update order but not registered.", group_name).c_str());
				continue;
			}

			if (group->update_type != type)
			{
				continue;
			}

			const bool serial = group->main_thread;
			bool overlap = overlapping_groups.find(group_name) != overlapping_groups.end();

			if (overlap && serial)
			{
				LOG_ERROR(std::format("Group {} is marked as overlapping but runs on the main thread, overlap ignored.", group_name).c_str());
				overlap = false;
			}

			if (overlap && stages.empty())
			{
				LOG_ERROR(std::format("Group {} is marked as overlapping but is the first stage of the pass, overlap ignored.", group_name).c_str());
				overlap = false;
			}

			if (overlap && stages.back().serial)
			{
				LOG_ERROR(std::format("Group {} is marked as overlapping but {} before it runs on the main thread, overlap ignored.", group_name, stages.back().group_names.back()).c_str());
				overlap = false;
			}

			if (!overlap)
			{
				stages.emplace_back();
				stages.back().serial = serial;
			}

			StageBuilder& stage = stages.back();
			stage.group_names.push_back(group_name);
			stage.member_names.insert(stage.member_names.end(), members_iter->second.begin(),
				members_iter->second.end());
		}

		std::unordered_map<std::string, size_t> stage_of_task;
		for (size_t stage_index = 0; stage_index < stages.size(); ++stage_index)
		{
			for (const std::string& task_name : stages[stage_index].member_names)
			{
				stage_of_task[task_name] = stage_index;
			}
		}

		// Only dependencies inside a stage constrain parallelism.
		std::vector<std::vector<std::pair<std::string, std::string>>> stage_edges(stages.size());

		for (const std::pair<std::string, std::string>& edge : edges)
		{
			const auto from_stage = stage_of_task.find(edge.first);
			const auto to_stage = stage_of_task.find(edge.second);

			if (from_stage == stage_of_task.end() || to_stage == stage_of_task.end())
			{
				// One of the two is filtered out by tag, or sits in a group that is not scheduled -
				// the latter is reported separately below.
				continue;
			}

			if (from_stage->second == to_stage->second)
			{
				stage_edges[from_stage->second].push_back(edge);
				continue;
			}

			if (from_stage->second > to_stage->second)
			{
				LOG_ERROR(std::format("Dependency {} -> {} cannot be satisfied: it points backwards through the update order.", edge.first, edge.second).c_str());
			}
		}

		ref<UpdateSchedule> schedule = make_ref<UpdateSchedule>();
		schedule->stages.reserve(stages.size());

		for (size_t stage_index = 0; stage_index < stages.size(); ++stage_index)
		{
			const StageBuilder& builder = stages[stage_index];
			const std::vector<std::string>& members = builder.member_names;
			const std::unordered_set<std::string> member_set(members.begin(), members.end());

			std::string stage_label;
			for (const std::string& group_name : builder.group_names)
			{
				if (!stage_label.empty())
				{
					stage_label += "+";
				}
				stage_label += group_name;
			}

			std::vector<std::pair<std::string, std::string>> local_edges;
			for (const std::pair<std::string, std::string>& edge : stage_edges[stage_index])
			{
				if (member_set.find(edge.first) == member_set.end() || member_set.find(edge.second) == member_set.end())
				{
					continue;
				}

				local_edges.push_back(edge);
			}

			// Deterministic topological order.
			const auto topological_order = [&members](const std::vector<std::pair<std::string, std::string>>& edge_list,
				std::vector<std::string>& out_order) -> bool {
					std::unordered_map<std::string, std::vector<std::string>> adjacency;
					std::unordered_map<std::string, int32> in_degree;
					adjacency.reserve(members.size());
					in_degree.reserve(members.size());

					for (const std::string& task_name : members)
					{
						adjacency.emplace(task_name, std::vector<std::string>{});
						in_degree.emplace(task_name, 0);
					}

					for (const std::pair<std::string, std::string>& edge : edge_list)
					{
						adjacency[edge.first].push_back(edge.second);
						++in_degree[edge.second];
					}

					std::vector<std::string> frontier;
					for (const std::string& task_name : members)
					{
						if (in_degree[task_name] == 0)
						{
							frontier.push_back(task_name);
						}
					}
					std::sort(frontier.begin(), frontier.end());

					out_order.clear();
					out_order.reserve(members.size());

					while (!frontier.empty())
					{
						std::vector<std::string> next_frontier;
						for (const std::string& task_name : frontier)
						{
							out_order.push_back(task_name);

							for (const std::string& neighbour : adjacency[task_name])
							{
								if (--in_degree[neighbour] == 0)
								{
									next_frontier.push_back(neighbour);
								}
							}
						}

						std::sort(next_frontier.begin(), next_frontier.end());
						frontier.swap(next_frontier);
					}

					return out_order.size() == members.size();
				};

			std::vector<std::string> order;
			if (!topological_order(local_edges, order))
			{
				const std::unordered_set<std::string> ordered(order.begin(), order.end());

				std::string cycle_members;
				for (const std::string& task_name : members)
				{
					if (ordered.find(task_name) != ordered.end())
					{
						continue;
					}

					if (!cycle_members.empty())
					{
						cycle_members += ", ";
					}
					cycle_members += task_name;
				}
				LOG_ERROR(std::format("Cycle in the task dependency graph, these tasks are scheduled last: {}", cycle_members).c_str());

				std::vector<std::pair<std::string, std::string>> acyclic_edges;
				for (const std::pair<std::string, std::string>& edge : local_edges)
				{
					if (ordered.find(edge.first) == ordered.end() && ordered.find(edge.second) == ordered.end())
					{
						continue;
					}

					acyclic_edges.push_back(edge);
				}
				local_edges.swap(acyclic_edges);

				const bool resolved = topological_order(local_edges, order);
				ASSERT(resolved);
			}

			StageSchedule stage_schedule;
			stage_schedule.name = stage_label;
			stage_schedule.group_names = builder.group_names;
			stage_schedule.serial = builder.serial;
			stage_schedule.nodes.reserve(order.size());

			std::unordered_map<std::string, uint32> node_index;
			node_index.reserve(order.size());

			for (const std::string& task_name : order)
			{
				node_index.emplace(task_name, static_cast<uint32>(stage_schedule.nodes.size()));

				StageNode node;
				node.task = source.at(task_name);
				stage_schedule.nodes.push_back(std::move(node));
			}

			for (const std::pair<std::string, std::string>& edge : local_edges)
			{
				const uint32 from = node_index.at(edge.first);
				const uint32 to = node_index.at(edge.second);

				stage_schedule.nodes[from].successors.push_back(to);
				++stage_schedule.nodes[to].predecessor_count;
			}

			for (uint32 index = 0; index < static_cast<uint32>(stage_schedule.nodes.size()); ++index)
			{
				if (stage_schedule.nodes[index].predecessor_count == 0)
				{
					stage_schedule.roots.push_back(index);
				}
			}

			if (!stage_schedule.serial && stage_schedule.nodes.size() > 1)
			{
				std::vector<uint32> depth(stage_schedule.nodes.size(), 1);
				uint32 critical_path = 1;

				for (size_t index = 0; index < stage_schedule.nodes.size(); ++index)
				{
					for (uint32 successor : stage_schedule.nodes[index].successors)
					{
						depth[successor] = std::max(depth[successor], depth[index] + 1);
						critical_path = std::max(critical_path, depth[successor]);
					}
				}

				if (critical_path == static_cast<uint32>(stage_schedule.nodes.size()))
				{
					LOG_ERROR(std::format("Concurrent stage {} is a single After/Before chain of {} tasks - nothing inside it can run in parallel.", stage_label, std::to_string(stage_schedule.nodes.size())).c_str());
				}
			}

			schedule->task_count += stage_schedule.nodes.size();
			schedule->stages.push_back(std::move(stage_schedule));
		}

		for (const auto& entry : group_task_names)
		{
			if (scheduled_groups.find(entry.first) == scheduled_groups.end())
			{
				LOG_ERROR(std::format("Group {} is not part of the update order, {} task(s) will never run.", entry.first, std::to_string(entry.second.size())).c_str());
			}
		}

		return schedule;
	}

	std::string WorldSystemScheduler::describe_schedule(UpdateType type) const
	{
		const UpdateScheduleRef schedule = get_schedule(type);

		std::string result = type == UpdateType::NORMAL ? std::string("NORMAL schedule") : std::string("FIXED schedule");

		if (!schedule)
		{
			result += ": not built yet\n";
			return result;
		}

		result += ": " + std::to_string(schedule->task_count) + " task(s), " +
			std::to_string(schedule->stages.size()) + " stage(s), " + std::to_string(workers.size()) +
			" worker(s) + the calling thread\n";

		for (const StageSchedule& stage : schedule->stages)
		{
			std::vector<uint32> depth(stage.nodes.size(), 1);
			uint32 critical_path = stage.nodes.empty() ? 0u : 1u;

			for (size_t index = 0; index < stage.nodes.size(); ++index)
			{
				for (uint32 successor : stage.nodes[index].successors)
				{
					depth[successor] = std::max(depth[successor], depth[index] + 1);
					critical_path = std::max(critical_path, depth[successor]);
				}
			}

			result += "  stage " + stage.name + (stage.serial ? " [serial]" : " [concurrent]");
			if (stage.group_names.size() > 1)
			{
				result += " (" + std::to_string(stage.group_names.size()) + " fused groups)";
			}
			result += ": " + std::to_string(stage.nodes.size()) + " task(s), " + std::to_string(stage.roots.size()) +
				" runnable immediately, critical path " + std::to_string(critical_path) + "\n";

			for (const StageNode& node : stage.nodes)
			{
				result += "    " + node.task->name;
				if (node.predecessor_count != 0)
				{
					result += " (waits for " + std::to_string(node.predecessor_count) + ")";
				}
				result += "\n";
			}
		}

		return result;
	}

	WorldSystemScheduler::Stats WorldSystemScheduler::get_stats() const
	{
		Stats stats;
		stats.last_fixed_step_ms = stat_last_fixed_step_ms.load(std::memory_order_relaxed);
		stats.max_fixed_step_ms = stat_max_fixed_step_ms.load(std::memory_order_relaxed);
		stats.fixed_steps = stat_fixed_steps.load(std::memory_order_relaxed);
		stats.fixed_overruns = stat_fixed_overruns.load(std::memory_order_relaxed);
		stats.dropped_fixed_steps = stat_dropped_fixed_steps.load(std::memory_order_relaxed);
		return stats;
	}

	void WorldSystemScheduler::reset_stats()
	{
		stat_last_fixed_step_ms.store(0.0, std::memory_order_relaxed);
		stat_max_fixed_step_ms.store(0.0, std::memory_order_relaxed);
		stat_fixed_steps.store(0, std::memory_order_relaxed);
		stat_fixed_overruns.store(0, std::memory_order_relaxed);
		stat_dropped_fixed_steps.store(0, std::memory_order_relaxed);
	}

	Task::Task(System* _system,
		const rttr::method& _method,
		const std::string& _group,
		const std::string& _tag,
		const std::vector<std::string>& _dependencies,
		const std::vector<std::string>& _dependents)
		: system(_system)
		, method(_method)
		, group(_group)
		, tag(_tag)
		, dependencies(_dependencies)
		, dependents(_dependents)
	{
		ASSERT(system != nullptr);

		const std::string type_name = system != nullptr ? to_std(system->get_type().get_name()) : std::string("<null>");
		name = type_name + "::" + to_std(_method.get_name());
	}

	void Task::invoke(float dt) const
	{
		ZoneScopedN("Task::invoke");

		if (system == nullptr || !method.is_valid())
		{
			return;
		}

		method.invoke(*system, dt);
	}
}
