#pragma once

#include "core_api.h"

#include "core/camera.h"
#include "core/math.h"

#include "ecs/component.h"

namespace era_engine
{

	class ERA_CORE_API CameraHolderComponent final : public Component
	{
	public:
		enum CameraType : uint8_t
		{
			FREE_CAMERA = 0,
			FREE_CAMERA_ON_HOLD,
			ATTACHED_TO_TRS,
			USER_DEFINED
		};

		CameraHolderComponent() = default;
		CameraHolderComponent(ref<Entity::EcsData> _data);

		~CameraHolderComponent() override;

		void set_render_camera(render_camera* _camera);
		render_camera* get_render_camera() const;

		CameraType get_camera_type() const;
		void set_camera_type(CameraType _type);

		ERA_VIRTUAL_REFLECT(Component)

	private:
		render_camera* camera = nullptr;

		CameraType type = ATTACHED_TO_TRS;
	};

}