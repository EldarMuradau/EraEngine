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

	// One update group, fully resolved at graph build time. waves[i] may run in parallel, waves[i + 1] starts only after waves[i] finished.
	struct GroupSchedule
	{
		std::string name;
		bool serial = false; // Run in order on the calling thread.
		std::vector<std::vector<ref<Task>>> waves;
	};

	// Immutable snapshot of a whole update pass.
	struct UpdateSchedule
	{
		std::vector<GroupSchedule> groups;
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
		WorldSystemScheduler(World* _world, size_t normal_threads = 0, size_t fixed_threads = 0);

		~WorldSystemScheduler();

		WorldSystemScheduler(const WorldSystemScheduler& _other) = delete;
		WorldSystemScheduler& operator=(const WorldSystemScheduler& _other) = delete;

		void stop();
		bool is_running() const;

		void set_fixed_update_rate(double rate_hz);
		double get_fixed_update_rate() const;

		void set_fixed_step_mode(FixedStepMode mode);
		FixedStepMode get_fixed_step_mode() const;

		void set_max_fixed_steps_per_wakeup(uint32 steps);

		void initialize_systems(const rttr::array_range<rttr::type>& types);

		void initialize_all_systems();

		void refresh_graph();

		void update_normal(float dt);

		void update_fixed(float dt);

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

		struct TaskItem
		{
			ref<Task> task;
			float dt = 0.0f;
		};

		void worker_loop();

		void fixed_timer_loop();

		void run_schedule(const UpdateScheduleRef& schedule, float dt, size_t kind);

		void run_wave(const std::vector<ref<Task>>& wave, float dt, size_t kind);

		bool has_pending_locked() const;
		bool pop_any_locked(TaskItem& out_item, size_t& out_kind);
		bool pop_from_locked(size_t kind, TaskItem& out_item);
		void on_task_finished(size_t kind);

		void add_task(ref<Task> task, UpdateType type);

		UpdateScheduleRef build_schedule(UpdateType type) const;
		UpdateScheduleRef get_schedule(UpdateType type) const;

		uint32 pump_fixed_steps();

		std::chrono::steady_clock::duration fixed_interval() const;

	protected:
		World* world = nullptr;

		// Single shared pool with one queue per update type. Workers alternate between the two
		// queues, so neither the normal frame nor the fixed step can starve the other.
		std::vector<std::thread> workers;
		std::deque<TaskItem> queues[QUEUE_COUNT];
		int32 in_flight[QUEUE_COUNT] = { 0, 0 };
		bool prefer_fixed = false;

		mutable std::mutex work_mutex;
		std::condition_variable work_available;
		std::condition_variable queue_idle;
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

		bool inited = false;
		bool systems_initialized = false;
	};
}
