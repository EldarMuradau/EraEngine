// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#include "rendering/light_source.h"

#include <rttr/registration>

namespace era_engine
{
	void directional_light::updateMatrices(const render_camera& camera)
	{
		camera_frustum_corners worldFrustum = camera.getWorldSpaceFrustumCorners(cascadeDistances.w);

		vec3 bottomLeftRay = worldFrustum.farBottomLeft - worldFrustum.nearBottomLeft;
		vec3 bottomRightRay = worldFrustum.farBottomRight - worldFrustum.nearBottomRight;
		vec3 topLeftRay = worldFrustum.farTopLeft - worldFrustum.nearTopLeft;
		vec3 topRightRay = worldFrustum.farTopRight - worldFrustum.nearTopRight;

		vec3 cameraForward = camera.rotation * vec3(0.f, 0.f, -1.f);

		bottomLeftRay /= dot(bottomLeftRay, cameraForward);
		bottomRightRay /= dot(bottomRightRay, cameraForward);
		topLeftRay /= dot(topLeftRay, cameraForward);
		topRightRay /= dot(topRightRay, cameraForward);

		for (uint32 cascade = 0; cascade < numShadowCascades; ++cascade)
		{
			float prevDistance = (cascade == 0) ? camera.nearPlane : (cascadeDistances.data[cascade - 1] - blendDistances.data[cascade - 1]);
			float distance = cascadeDistances.data[cascade];

			vec3 corners[8];
			corners[0] = camera.position + bottomLeftRay * prevDistance;
			corners[1] = camera.position + bottomRightRay * prevDistance;
			corners[2] = camera.position + topLeftRay * prevDistance;
			corners[3] = camera.position + topRightRay * prevDistance;
			corners[4] = camera.position + bottomLeftRay * distance;
			corners[5] = camera.position + bottomRightRay * distance;
			corners[6] = camera.position + topLeftRay * distance;
			corners[7] = camera.position + topRightRay * distance;

			vec3 center(0.0f);
			for (uint32 i = 0; i < 8; ++i)
			{
				center += corners[i];
			}
			center /= 8.0f;

			vec3 upDir = vec3(0.f, 1.0f, 0.f);
			if (abs(dot(upDir, direction)) > 1.0f - EPSILON)
			{
				upDir = vec3(0.0f, 0.0f, 1.0f);
			}

			mat4 viewMatrix = look_at(center, center + direction, upDir);

			vec3 minExtents, maxExtents;
			if (stabilize)
			{
				vec3 viewCenter = transform_position(viewMatrix, center);

				float sphereRadius = 0.f;
				for (uint32 i = 0; i < 8; ++i)
				{
					float d = length(transform_position(viewMatrix, corners[i]) - viewCenter);
					sphereRadius = max(sphereRadius, d);
				}

				sphereRadius = ceil(sphereRadius * 16.0f) / 16.0f;
				minExtents = -sphereRadius;
				maxExtents = sphereRadius;
			}
			else
			{
				bounding_box extents = bounding_box::negativeInfinity();
				for (uint32 i = 0; i < 8; ++i)
				{
					vec3 c = transform_position(viewMatrix, corners[i]);
					extents.grow(c);
				}

				minExtents = extents.minCorner;
				maxExtents = extents.maxCorner;
			}

			vec3 cascadeExtents = maxExtents - minExtents;
			vec3 shadowCamPos = center + direction * minExtents.z;

			//viewMatrix = lookAt(shadowCamPos, shadowCamPos + direction, upDir);

			mat4 projMatrix = create_orthographic_projection_matrix(maxExtents.x, minExtents.x, maxExtents.y, minExtents.y, -negativeZOffset, cascadeExtents.z);

			{
				mat4 matrix = projMatrix * viewMatrix;
				vec3 shadowOrigin(0.0f);
				shadowOrigin = (matrix * vec4(shadowOrigin, 1.0f)).xyz;
				shadowOrigin *= (shadowDimensions * 0.5f);

				vec3 roundedOrigin = round(shadowOrigin);
				vec3 roundOffset = roundedOrigin - shadowOrigin;
				roundOffset = roundOffset * (2.f / shadowDimensions);

				projMatrix.m03 += roundOffset.x;
				projMatrix.m13 += roundOffset.y;
			}

			viewProjs[cascade] = projMatrix * viewMatrix;
		}
	}

	mat4 getSpotLightViewProjectionMatrix(const spot_light_cb& sl)
	{
		mat4 viewMatrix = look_at(sl.position, sl.position + sl.direction, vec3(0.f, 1.f, 0.f));
		mat4 projMatrix = create_perspective_projection_matrix(acos(sl.getOuterCutoff()) * 2.f, 1.f, 0.01f, sl.maxDistance);
		return projMatrix * viewMatrix;
	}

	RTTR_REGISTRATION
	{
		using namespace rttr;
		rttr::registration::class_<directional_light>("directional_light")
			.constructor<>()
			.property("color", &directional_light::color)
			.property("intensity", &directional_light::intensity)
			.property("bias", &directional_light::bias)
			.property("cascadeDistances", &directional_light::cascadeDistances)
			.property("blendDistances", &directional_light::blendDistances)
			.property("direction", &directional_light::direction)
			.property("negativeZOffset", &directional_light::negativeZOffset)
			.property("shadowDimensions", &directional_light::shadowDimensions)
			.property("numShadowCascades", &directional_light::numShadowCascades)
			.property("stabilize", &directional_light::stabilize);

		rttr::registration::class_<PointLightComponent>("PointLightComponent")
			.constructor<>()
			.property("color", &PointLightComponent::color)
			.property("intensity", &PointLightComponent::intensity)
			.property("radius", &PointLightComponent::radius)
			.property("castsShadow", &PointLightComponent::castsShadow)
			.property("shadowMapResolution", &PointLightComponent::shadowMapResolution);

		rttr::registration::class_<SpotLightComponent>("SpotLightComponent")
			.constructor<>()
			.property("color", &SpotLightComponent::color)
			.property("intensity", &SpotLightComponent::intensity)
			.property("distance", &SpotLightComponent::distance)
			.property("innerAngle", &SpotLightComponent::innerAngle)
			.property("outerAngle", &SpotLightComponent::outerAngle)
			.property("castsShadow", &SpotLightComponent::castsShadow)
			.property("shadowMapResolution", &SpotLightComponent::shadowMapResolution);
	}

	PointLightComponent::PointLightComponent(ref<Entity::EcsData> _data, const vec3& _color, float _intensity, float _radius, bool _castsShadow, uint32 _shadowMapResolution)
		: Component(_data), color(_color), intensity(_intensity), radius(_radius), castsShadow(_castsShadow), shadowMapResolution(_shadowMapResolution)
	{
	}

	PointLightComponent::~PointLightComponent()
	{
	}

	SpotLightComponent::SpotLightComponent(ref<Entity::EcsData> _data, const vec3& _color, float _intensity, float _distance, float _innerAngle, float _outerAngle, bool _castsShadow, uint32 _shadowMapResolution)
		: Component(_data), color(_color), intensity(_intensity), distance(_distance), innerAngle(_innerAngle), outerAngle(_outerAngle), castsShadow(_castsShadow), shadowMapResolution(_shadowMapResolution)

	{
	}

	SpotLightComponent::~SpotLightComponent()
	{
	}

}