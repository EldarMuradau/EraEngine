#pragma once

#include "core_api.h"

#include "ecs/system.h"

#include "core/job_system.h"
#include "core/sync.h"

#include <rttr/type>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace era_engine
{
	class World;

	struct ERA_CORE_API Task
	{
		Task(System* _system,
			const rttr::method& _method,
			const std::string& _group,
			const std::string& _tag,
			const std::vector<std::string>& _dependencies,
			const std::vector<std::string>& _dependents);

		Task(const Task& _other) = default;
		Task(Task&& _other) noexcept = default;

		void invoke(float dt) const;

		System* system = nullptr;
		rttr::method method;

		// SystemType::method_name.
		std::string name;
		std::string group;
		std::string tag;
		std::vector<std::string> dependencies;
		std::vector<std::string> dependents;
	};

	// One task inside a stage, with its intra-stage dependencies already resolved.
	struct StageNode
	{
		ref<Task> task;

		std::vector<uint32> successors;

		// How many tasks of the same stage must finish before this one may start.
		uint32 predecessor_count = 0;
	};

	struct StageSchedule
	{
		std::string name;
		std::vector<std::string> group_names;
		bool serial = false;

		// Topologically ordered, which is also the order used when the stage runs inline.
		std::vector<StageNode> nodes;

		// Nodes with predecessor_count == 0: the tasks dispatched when the stage starts.
		std::vector<uint32> roots;
	};

	// Immutable snapshot of a whole update pass.
	struct UpdateSchedule
	{
		std::vector<StageSchedule> stages;
		size_t task_count = 0;
	};

	using UpdateScheduleRef = ref<const UpdateSchedule>;

	enum class FixedStepMode
	{
		// Fixed steps are driven by a dedicated timer thread.
		// Fixed and normal systems run concurrently.
		THREADED,

		// Fixed steps are pumped from update_normal(). Deterministic, no normal/fixed data races
		// and no cross-thread contention at all.
		MAIN_THREAD_PUMP
	};

	class ERA_CORE_API WorldSystemScheduler
	{
	public:
		// `normal_threads` and `fixed_threads` are summed into a *single* shared pool. Two
		// independent pools oversubscribe the machine and let fixed work preempt the frame.
		// 0 (default) sizes the pool from hardware_concurrency.
		WorldSystemScheduler(World* _world, size_t normal_threads = 0, size_t fixed_threads = 0);

		~WorldSystemScheduler();

		WorldSystemScheduler(const WorldSystemScheduler& _other) = delete;
		WorldSystemScheduler& operator=(const WorldSystemScheduler& _other) = delete;

		void stop();
		bool is_running() const;

		// `rate_hz` is a frequency (30.0 == 30 steps per second).
		void set_fixed_update_rate(double rate_hz);
		double get_fixed_update_rate() const;

		void set_fixed_step_mode(FixedStepMode mode);
		FixedStepMode get_fixed_step_mode() const;

		void set_max_fixed_steps_per_wakeup(uint32 steps);

		void initialize_systems(const rttr::array_range<rttr::type>& types);

		void initialize_all_systems();

		void refresh_graph();

		// Removes the barrier between `group_name` and the group that precedes it in the update
		// order: both run as a single stage and only real After/Before dependencies between their
		// tasks are enforced.
		void set_group_overlap(const std::string& group_name, bool may_overlap_previous);

		void set_overlapping_groups(const std::vector<std::string>& group_names);

		bool get_group_overlap(const std::string& group_name) const;

		void update_normal(float dt);

		void update_fixed(float dt);

		std::string describe_schedule(UpdateType type) const;

		struct Stats
		{
			double last_fixed_step_ms = 0.0;
			double max_fixed_step_ms = 0.0;
			uint64 fixed_steps = 0;
			uint64 fixed_overruns = 0;
			uint64 dropped_fixed_steps = 0;
		};

		Stats get_stats() const;
		void reset_stats();

	protected:
		enum QueueKind : size_t
		{
			QUEUE_NORMAL = 0,
			QUEUE_FIXED = 1,
			QUEUE_COUNT = 2
		};

		// Live state of one stage execution.
		struct StageRun
		{
			const StageSchedule* stage = nullptr;
			std::vector<uint32> remaining;
			float dt = 0.0f;
			uint32 completed = 0;
			uint32 total = 0;
		};

		struct TaskItem
		{
			StageRun* run = nullptr;
			uint32 node = 0;
		};

		void worker_loop();

		void fixed_timer_loop();

		void run_schedule(const UpdateScheduleRef& schedule, float dt, size_t kind);

		// Dispatches the stage as a dataflow graph and keeps executing its tasks on the calling
		// thread until the stage is done, instead of parking on a barrier.
		void run_stage(const StageSchedule& stage, float dt, size_t kind);

		// Invokes one node, then releases the successors it was blocking.
		void execute_node(const TaskItem& item, size_t kind);

		bool has_pending_locked() const;
		bool pop_any_locked(TaskItem& out_item, size_t& out_kind);
		bool pop_from_locked(size_t kind, TaskItem& out_item);

		void add_task(ref<Task> task, UpdateType type);

		UpdateScheduleRef build_schedule(UpdateType type) const;
		UpdateScheduleRef get_schedule(UpdateType type) const;

		uint32 pump_fixed_steps();

		std::chrono::steady_clock::duration fixed_interval() const;

	protected:
		World* world = nullptr;

		std::vector<std::thread> workers;
		std::deque<TaskItem> queues[QUEUE_COUNT];
		bool prefer_fixed = false;

		mutable std::mutex work_mutex;
		std::condition_variable work_available;
		std::atomic<bool> running = false;

		std::thread fixed_timer_thread;
		std::mutex fixed_step_mutex;
		std::atomic<double> fixed_update_rate = 30.0;
		std::atomic<uint32> max_fixed_steps_per_wakeup = 4;
		std::atomic<FixedStepMode> fixed_step_mode = FixedStepMode::THREADED;

		// Owned by the timer thread.
		std::chrono::steady_clock::time_point next_fixed_update{};
		std::chrono::steady_clock::duration fixed_spin_margin = std::chrono::microseconds(300);

		// Owned by the thread calling update_normal().
		std::chrono::steady_clock::time_point next_pumped_fixed_update{};
		bool pump_clock_valid = false;

		std::atomic<double> stat_last_fixed_step_ms = 0.0;
		std::atomic<double> stat_max_fixed_step_ms = 0.0;
		std::atomic<uint64> stat_fixed_steps = 0;
		std::atomic<uint64> stat_fixed_overruns = 0;
		std::atomic<uint64> stat_dropped_fixed_steps = 0;

		std::vector<System*> systems;
		std::set<rttr::type> system_types;

		mutable std::mutex schedule_mutex;
		UpdateScheduleRef normal_schedule;
		UpdateScheduleRef fixed_schedule;

		std::unordered_map<std::string, ref<Task>> tasks;
		std::unordered_map<std::string, ref<Task>> fixed_tasks;

		std::set<std::string> overlapping_groups;

		bool inited = false;
		bool systems_initialized = false;
	};
}
