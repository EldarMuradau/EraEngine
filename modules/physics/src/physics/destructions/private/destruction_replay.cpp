#include "physics/destructions/destruction_replay.h"

#include "toolkit/NvBlastTk.h"

#include "physics/destructions/blast_physx/NvBlastExtPxManager.h"
#include "physics/destructions/blast_physx/NvBlastExtPxFamily.h"

namespace era_engine::physics
{
	DestructionReplay::DestructionReplay()
	{
		using namespace Nv::Blast;

		sync = ExtSync::create();
		reset();
	}

	DestructionReplay::~DestructionReplay()
	{
		sync->release();
		clear_buffer();
	}

	void DestructionReplay::add_family(Nv::Blast::TkFamily* family)
	{
		family->addListener(*sync);
	}

	void DestructionReplay::remove_family(Nv::Blast::TkFamily* family)
	{
		family->removeListener(*sync);
	}

	bool DestructionReplay::has_record() const
	{
		return buffer.size() > 0;
	}

	size_t DestructionReplay::get_event_count() const
	{
		return is_recording ? sync->getSyncBufferSize() : buffer.size();
	}

	uint32 DestructionReplay::get_current_event_index() const
	{
		return next_event_index;
	}

	void DestructionReplay::start_recording(Nv::Blast::ExtPxManager& manager, bool sync_family, bool sync_physics)
	{
		using namespace Nv::Blast;

		if (is_recording)
		{
			return;
		}

		sync->releaseSyncBuffer();

		if (sync_family || sync_physics)
		{
			std::vector<ExtPxFamily*> families(manager.getFamilyCount());

			manager.getFamilies(families.data(), (uint32)families.size());

			for (ExtPxFamily* family : families)
			{
				if (sync_physics)
				{
					sync->syncFamily(*family);
				}
				else if (sync_family)
				{
					sync->syncFamily(family->getTkFamily());
				}
			}
		}

		is_recording = true;
	}

	void DestructionReplay::stop_recording()
	{
		using namespace Nv::Blast;

		if (!is_recording)
		{
			return;
		}

		const ExtSyncEvent* const* temp_buffer = nullptr;
		uint32 size = 0;
		sync->acquireSyncBuffer(temp_buffer, size);

		clear_buffer();
		buffer.resize(size);
		for (uint32 i = 0; i < size; ++i)
		{
			buffer[i] = temp_buffer[i]->clone();
		}

		// TODO: sort by ts ? make sure?
		//buffer.sort

		sync->releaseSyncBuffer();

		is_recording = false;
	}

	void DestructionReplay::start_playback(Nv::Blast::ExtPxManager& manager, Nv::Blast::TkGroup* tk_group)
	{
		if (is_playing || !has_record())
		{
			return;
		}

		is_playing = true;
		start_time = std::chrono::steady_clock::now();
		next_event_index = 0;
		first_event_ts = buffer[0]->timestamp;
		px_manager = &manager;
		group = tk_group;
	}

	void DestructionReplay::stop_playback()
	{
		if (!is_playing)
		{
			return;
		}

		is_playing = false;
		px_manager = nullptr;
		group = nullptr;
	}

	void DestructionReplay::update()
	{
		using namespace std::chrono;
		using namespace Nv::Blast;

		if (is_playing)
		{
			auto now = steady_clock::now();
			auto mil = duration_cast<milliseconds>((now - start_time));
			bool stop = true;

			while (next_event_index < buffer.size())
			{
				const ExtSyncEvent* current_event = buffer[next_event_index];
				auto dt = current_event->timestamp - first_event_ts;
				if (dt < (uint64)mil.count())
				{
					sync->applySyncBuffer(px_manager->getFramework(), &current_event, 1, group, px_manager);
					next_event_index++;
				}
				else
				{
					stop = false;
					break;
				}
			}

			if (stop)
			{
				stop_playback();
			}
		}
	}

	void DestructionReplay::reset()
	{
		is_playing = false;
		is_recording = false;
		sync->releaseSyncBuffer();
	}

	void DestructionReplay::clear_buffer()
	{
		for (auto element : buffer)
		{
			element->release();
		}
		buffer.clear();
	}

}