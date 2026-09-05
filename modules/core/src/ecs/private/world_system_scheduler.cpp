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
		queue_idle.notify_all();

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
			in_flight[kind] = 0;
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

		refresh_graph();

		for (System* system : systems)
		{
			system->init();
		}

		refresh_graph();

		systems_initialized = true;

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

		run_schedule(get_schedule(UpdateType::FIXED), world->get_fixed_update_dt(), QUEUE_FIXED);

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

		for (const GroupSchedule& group : schedule->groups)
		{
			if (!running.load(std::memory_order_relaxed))
			{
				return;
			}

			for (const std::vector<ref<Task>>& wave : group.waves)
			{
				if (group.serial)
				{
					for (const ref<Task>& task : wave)
					{
						task->invoke(dt);
					}
				}
				else
				{
					run_wave(wave, dt, kind);
				}
			}
		}
	}

	void WorldSystemScheduler::run_wave(const std::vector<ref<Task>>& wave, float dt, size_t kind)
	{
		if (wave.empty())
		{
			return;
		}

		if (wave.size() == 1 || workers.empty())
		{
			for (const ref<Task>& task : wave)
			{
				task->invoke(dt);
			}
			return;
		}

		{
			std::lock_guard<std::mutex> lock(work_mutex);
			for (size_t i = 0; i + 1 < wave.size(); ++i)
			{
				queues[kind].push_back(TaskItem{ wave[i], dt });
			}
		}
		work_available.notify_all();

		wave.back()->invoke(dt);

		while (true)
		{
			TaskItem item;
			{
				std::unique_lock<std::mutex> lock(work_mutex);
				if (!pop_from_locked(kind, item))
				{
					queue_idle.wait(lock, [this, kind] {
						return !running.load(std::memory_order_relaxed) ||
							(queues[kind].empty() && in_flight[kind] == 0);
						});
					return;
				}
			}

			item.task->invoke(item.dt);
			on_task_finished(kind);
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

		out_item = std::move(queues[kind].front());
		queues[kind].pop_front();
		++in_flight[kind];

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

	void WorldSystemScheduler::on_task_finished(size_t kind)
	{
		bool became_idle = false;
		{
			std::lock_guard<std::mutex> lock(work_mutex);
			--in_flight[kind];
			became_idle = in_flight[kind] == 0 && queues[kind].empty();
		}

		if (became_idle)
		{
			queue_idle.notify_all();
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

				if (!running.load(std::memory_order_relaxed))
				{
					break;
				}

				if (!pop_any_locked(item, kind))
				{
					continue;
				}
			}

			item.task->invoke(item.dt);
			on_task_finished(kind);
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

		std::vector<std::string> names;
		names.reserve(source.size());
		for (const auto& entry : source)
		{
			names.push_back(entry.first);
		}

		std::sort(names.begin(), names.end());

		std::unordered_map<std::string, std::vector<std::string>> adjacency;
		std::unordered_map<std::string, int32> in_degree;
		adjacency.reserve(names.size());
		in_degree.reserve(names.size());

		for (const std::string& task_name : names)
		{
			adjacency.emplace(task_name, std::vector<std::string>{});
			in_degree.emplace(task_name, 0);
		}

		std::set<std::pair<std::string, std::string>> edges;

		const auto add_edge = [&](const std::string& from, const std::string& to, const char* relation) {
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

			if (!edges.emplace(from, to).second)
			{
				return;
			}

			adjacency[from].push_back(to);
			++in_degree[to];
			};

		for (const std::string& task_name : names)
		{
			const ref<Task>& task = source.at(task_name);
			for (const std::string& dependency : task->dependencies)
			{
				add_edge(dependency, task_name, "After");
			}
			for (const std::string& dependent : task->dependents)
			{
				add_edge(task_name, dependent, "Before");
			}
		}

		std::unordered_map<std::string, uint32> levels;
		levels.reserve(names.size());

		std::vector<std::string> frontier;
		for (const std::string& task_name : names)
		{
			if (in_degree[task_name] == 0)
			{
				frontier.push_back(task_name);
			}
		}

		uint32 level = 0;
		size_t resolved = 0;
		while (!frontier.empty())
		{
			std::vector<std::string> next_frontier;

			for (const std::string& task_name : frontier)
			{
				levels[task_name] = level;
				++resolved;

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
			++level;
		}

		if (resolved != names.size())
		{
			std::string cycle_members;
			for (const std::string& task_name : names)
			{
				if (levels.find(task_name) == levels.end())
				{
					if (!cycle_members.empty())
					{
						cycle_members += ", ";
					}
					cycle_members += task_name;
					levels[task_name] = level;
				}
			}

			LOG_ERROR(std::format("Cycle in the task dependency graph, these tasks are scheduled last: {}", cycle_members).c_str());
			ASSERT(false);
		}

		std::unordered_map<std::string, std::vector<ref<Task>>> group_members;
		for (const std::string& task_name : names)
		{
			const ref<Task>& task = source.at(task_name);

			if (!world->has_tag(task->tag))
			{
				continue;
			}

			group_members[task->group].push_back(task);
		}

		ref<UpdateSchedule> schedule = make_ref<UpdateSchedule>();

		std::unordered_set<std::string> scheduled_groups;
		for (const std::string& group_name : UpdatesHolder::update_order)
		{
			if (group_name.empty())
			{
				continue;
			}

			if (!scheduled_groups.insert(group_name).second)
			{
				continue;
			}

			const auto members_iter = group_members.find(group_name);
			if (members_iter == group_members.end() || members_iter->second.empty())
			{
				continue;
			}

			UpdateGroup* group = find_group(group_name);
			if (group == nullptr)
			{
				LOG_WARNING(std::format("Update group {} is in the update order but not registered.", group_name).c_str());
				continue;
			}

			if (group->update_type != type)
			{
				continue;
			}

			GroupSchedule group_schedule;
			group_schedule.name = group_name;
			group_schedule.serial = group->main_thread;

			std::vector<ref<Task>> members = members_iter->second;
			std::stable_sort(members.begin(), members.end(), [&levels](const ref<Task>& a, const ref<Task>& b) {
				return levels.at(a->name) < levels.at(b->name);
				});

			for (const ref<Task>& task : members)
			{
				const uint32 task_level = levels.at(task->name);

				if (group_schedule.waves.empty() || levels.at(group_schedule.waves.back().front()->name) != task_level)
				{
					group_schedule.waves.emplace_back();
				}

				group_schedule.waves.back().push_back(task);
			}

			schedule->task_count += members.size();
			schedule->groups.push_back(std::move(group_schedule));
		}

		for (const auto& entry : group_members)
		{
			if (scheduled_groups.find(entry.first) == scheduled_groups.end())
			{
				LOG_WARNING(std::format("Group {} is not part of the update order, {} task(s) will never run.", entry.first, entry.second.size()).c_str());
			}
		}

		return schedule;
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
