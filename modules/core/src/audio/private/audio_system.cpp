#include "audio/audio_system.h"
#include "audio/audio.h"

#include "ecs/update_groups.h"

#include <rttr/policy.h>
#include <rttr/registration>

namespace era_engine
{
	RTTR_REGISTRATION
	{
		using namespace rttr;

		registration::class_<AudioSystem>("AudioSystem")
			.constructor<World*>()(policy::ctor::as_raw_ptr, metadata("Tag", std::string("base")))
			.method("update", &AudioSystem::update)(metadata("update_group", update_types::GAMEPLAY_NORMAL));
	}

	AudioSystem::AudioSystem(World* _world)
		: System(_world)
	{
		//initializeAudio();
	}

	AudioSystem::~AudioSystem()
	{
		//shutdownAudio();
	}

	void AudioSystem::init()
	{
	}

	void AudioSystem::update(float dt)
	{
		//updateAudio(dt);
	}

}