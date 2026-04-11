#pragma once

#include "core_api.h"

#include "ecs/system.h"

#include "core/job_system.h"
#include "core/sync.h"

#include <rttr/type>

#include <string>
#include <vector>
#include <unordered_map>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <chrono>
#include <functional>

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
		Task(Task&& _other) noexcept = default;
		Task(const Task& _other) noexcept = default;

		Task& operator=(const Task& _other) noexcept;
		Task& operator=(Task&& _other) noexcept;

		System* system = nullptr;
		rttr::method method;
		std::string group;
		std::string tag;
		std::vector<std::string> dependencies;
		std::vector<std::string> dependents;
	};

    class ERA_CORE_API WorldSystemScheduler
    {
    public:
        WorldSystemScheduler(World* _world, size_t normal_threads = 2, size_t fixed_threads = 2);

        ~WorldSystemScheduler();

        void stop();

        void set_fixed_update_rate(double rate);

        void initialize_systems(const rttr::array_range<rttr::type>& types);

        void initialize_all_systems();

        void refresh_graph();

        void update_normal(float dt);

        void update_fixed(float dt);

    protected:
        struct TaskItem 
        {
            ref<Task> task;
            float dt = 0.0f;
        };

        void normal_worker();

        void fixed_worker();
     
        void fixed_update_loop();

        std::unordered_map<std::string, std::vector<ref<Task>>> build_task_order(UpdateType type);

        void add_task(ref<Task> task, UpdateType type);

        const std::vector<ref<Task>>& get_fixed_group_tasks(const std::string& name) const;
        const std::vector<ref<Task>>& get_group_tasks(const std::string& name) const;

    protected:
        std::vector<std::thread> normal_thread_pool;
        std::queue<TaskItem> normal_task_queue;
        std::atomic<int> active_normal_tasks = 0;
        std::condition_variable tasks_done_cv;

        std::atomic<int> active_fixed_tasks = 0;
        std::condition_variable fixed_tasks_done_cv;

        std::vector<std::thread> fixed_thread_pool;
        std::queue<TaskItem> fixed_task_queue;

        std::mutex queue_mutex;
        std::mutex fixed_queue_mutex;
        std::condition_variable normal_condition;
        std::condition_variable fixed_condition;
        std::atomic<bool> running = false;

        std::thread fixed_update_thread;
        double fixed_update_rate = 1.0 / 30.0;
        std::chrono::duration<double> fixed_update_interval;
        std::chrono::time_point<std::chrono::steady_clock> last_fixed_update;

        std::vector<System*> systems;
        std::set<rttr::type> system_types;

        mutable SpinLock fixed_group_cycle_sync;
        mutable SpinLock group_cycle_sync;

        std::unordered_map<std::string, ref<Task>> tasks;
        std::unordered_map<std::string, std::vector<std::string>> adj_list;
        std::unordered_map<std::string, int32> in_degree;

        std::unordered_map<std::string, ref<Task>> fixed_tasks;
        std::unordered_map<std::string, std::vector<std::string>> fixed_adj_list;
        std::unordered_map<std::string, int32> fixed_in_degree;

        std::unordered_map<std::string, std::vector<ref<Task>>> grouped_ordered_tasks;
        std::unordered_map<std::string, std::vector<ref<Task>>> fixed_grouped_ordered_tasks;

        World* world = nullptr;
        bool inited = false;
    };
}