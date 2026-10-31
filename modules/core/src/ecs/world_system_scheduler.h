#pragma once

#include "core_api.h"

#include "ecs/system.h"

#include "core/job_system.h"
#include "core/sync.h"

#include <rttr/type>

#include <atomic>
#include <chrono>
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
        explicit WorldSystemScheduler(World* _world);

        ~WorldSystemScheduler();

        WorldSystemScheduler(const WorldSystemScheduler& _other) = delete;
        WorldSystemScheduler& operator=(const WorldSystemScheduler& _other) = delete;

        void stop();
        bool is_running() const;

        // 'rate_hz' is a frequency (30.0 == 30 steps per second).
        void set_fixed_update_rate(double rate_hz);
        double get_fixed_update_rate() const;

        void set_fixed_step_mode(FixedStepMode mode);
        FixedStepMode get_fixed_step_mode() const;

        void set_max_fixed_steps_per_wakeup(uint32 steps);

        void initialize_systems(const rttr::array_range<rttr::type>& types);

        void initialize_all_systems();

        void refresh_graph();

        // Removes the barrier between `group_name` and the group that precedes it in the update order.
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

        uint32 pump_fixed_steps();

    protected:
        struct StageRun;

        struct NodeJob
        {
            StageRun* run = nullptr;
            uint32 node = 0;
        };

        // Live state of one stage execution.
        struct StageRun
        {
            const StageSchedule* stage = nullptr;
            std::unique_ptr<std::atomic<uint32>[]> remaining;
            std::vector<NodeJob> node_jobs;
            float dt = 0.0f;
            JobCounter counter;
        };

        void fixed_timer_loop();
        void start_fixed_timer_thread();

        void run_schedule(const UpdateScheduleRef& schedule, float dt);

        // Dispatches the stage as a dataflow graph of jobs. The calling fiber waits for all of them.
        void run_stage(const StageSchedule& stage, float dt);

        static JobDecl make_node_job(StageRun& run, uint32 node);

        // Invokes one node, then dispatches the successors it was blocking.
        static void execute_node(void* node_job);

        void add_task(ref<Task> task, UpdateType type);

        UpdateScheduleRef build_schedule(UpdateType type) const;
        UpdateScheduleRef get_schedule(UpdateType type) const;

        std::chrono::steady_clock::duration fixed_interval() const;
        float compute_fixed_update_elapsed_share(std::chrono::steady_clock::time_point next_update) const;

    protected:
        World* world = nullptr;

        std::atomic<bool> running = false;

        // Started only for FixedStepMode::THREADED.
        std::thread fixed_timer_thread;
        std::mutex fixed_step_mutex;
        std::atomic<double> fixed_update_rate = 30.0;
        std::atomic<uint32> max_fixed_steps_per_wakeup = 4;
        std::atomic<FixedStepMode> fixed_step_mode = FixedStepMode::MAIN_THREAD_PUMP;

        // Owned by the timer thread.
        std::chrono::steady_clock::time_point next_fixed_update{};
        std::chrono::steady_clock::duration fixed_spin_margin = std::chrono::microseconds(300);

        // next_fixed_update published for the normal update.
        std::atomic<std::chrono::steady_clock::rep> next_fixed_update_ticks = 0;

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
