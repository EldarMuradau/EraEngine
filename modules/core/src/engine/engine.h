#pragma once

#include "core_api.h"

#include <rttr/type>

#include <functional>

namespace era_engine
{

    class ERA_CORE_API Engine final
    {
        Engine(int argc, char** argv);
        ~Engine();

    public:
        Engine(const Engine&) = delete;
        Engine& operator=(const Engine&) const = delete;

        static Engine* instance();
        static void release();

        static Engine* create_instance(int argc, char** argv);

        bool run(const std::function<void(void)>& initial_task = nullptr);

        void terminate();

        template <typename Object>
        Object* get(std::vector<rttr::argument> args = std::vector<rttr::argument>())
        {
            rttr::type type = rttr::type::get<Object>();
            Engine* engine = instance();

            auto iter = engine->single_objects.find(type);
            if (iter != engine->single_objects.end())
            {
                return iter->second.get_value<Object*>();
            }

            rttr::variant object = type.create(args);
            engine->single_objects.emplace(type, object);
            return object.get_value<Object*>();
        }

    private:
        bool update();

        bool running = false;

        std::unordered_map<rttr::type, rttr::variant> single_objects;

        static Engine* instance_object;
    };

    // Waits until the in-flight frame execution (command recording, present) is done. Main thread only.
    ERA_CORE_API void wait_for_frame_execution();

    template <typename Object>
    inline Object* get_object(std::vector<rttr::argument> args = std::vector<rttr::argument>())
    {
        return Engine::instance()->get<Object>(args);
    }

    template <typename Object>
    inline Object* get_transient_object(std::vector<rttr::argument> args = std::vector<rttr::argument>())
    {
        rttr::type type = rttr::type::get<Object>();
        rttr::variant object = type.create(args);
        return object.get_value<Object*>();
    }
}