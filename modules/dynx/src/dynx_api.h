#pragma once

#ifdef ERA_DYNX
#define ERA_DYNX_API __declspec(dllexport)
#else
#define ERA_DYNX_API __declspec(dllimport)
#endif 