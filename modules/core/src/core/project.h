#pragma once

#include "core_api.h"

namespace era_engine
{
	enum class ConfigurationType
	{
		Debug,
		Release
	};

	struct ERA_CORE_API Project
	{
		static inline std::wstring engine_path = ENGINE_PATH;
		static inline std::wstring path = engine_path + L"/projects/Era Engine New Proj";
		static inline std::wstring dotnet_path = engine_path + L"/resources/bin/Dotnet/Release/net8.0/";
		static inline std::wstring exe_path = L"";
		static inline std::wstring name = L"NewProject";

#if defined(_DEBUG)
		static inline ConfigurationType config = ConfigurationType::Debug;
#else
		static inline ConfigurationType config = ConfigurationType::Release;
#endif
	};
}