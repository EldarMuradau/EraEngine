#include "ecs/reflection_utils.h"

namespace era_engine
{
	std::string_view ReflectionUtils::extract_class_name(std::string_view name)
	{
		auto start = name.find_last_of(':');

		if (start == std::string_view::npos)
		{
			return {};
		}
		++start;

		auto end = name.find_first_of('>', start);
		if (end == std::string_view::npos)
		{
			return {};
		}

		return name.substr(start, end);
	}
}