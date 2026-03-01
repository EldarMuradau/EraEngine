#pragma once

#include "physics/physx_api.h"

#include <functional>

namespace era_engine::physics
{
	namespace helper
	{
		template <int... Is>
		struct Index {};

		template <int N, int... Is>
		struct GenSeq : GenSeq<N - 1, N - 1, Is...> {};

		template <int... Is>
		struct GenSeq<0, Is...> : Index<Is...> {};
	}

	template<typename Func, typename... Args>
	class Task : public physx::PxLightCpuTask
	{
		Task() = default;

		template<IsCallableFunc<Func, Args...> = true>
		Task(std::function<Func(Args...)> f, Args&&... ars)
			: func(f),
			args(std::forward<Args>(ars)...) {}

		~Task() {}

		const char* get_name() const override { return "PhysX Task"; }
		void run() override { call(args); }

	private:
		template <typename... Args, int... Is>
		void call(std::tuple<Args...>& tup, helper::Index<Is...>)
		{
			func(std::get<Is>(tup)...);
		}

		template <typename... Args>
		void call(std::tuple<Args...>& tup)
		{
			call(tup, helper::GenSeq<sizeof...(Args)>{});
		}

	protected:
		std::tuple<Args...> args;
		std::function<Func(Args...)> func;
	};
}