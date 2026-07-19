// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#pragma once

#include "core/camera.h"
#include "core/camera_controller.h"

#include "dx/dx_texture.h"

#include "rendering/main_renderer.h"

#include "ecs/editor/editor_scene.h"

namespace era_engine
{
	struct editor_scene;

	struct asset_editor_panel
	{
		virtual void beginFrame();
		virtual void endFrame() {}

		bool isOpen() const { return windowOpen; }
		void open();
		void close();

	private:
		virtual void edit(uint32 renderWidth, uint32 renderHeight) = 0;
		virtual ref<dx_texture> getRendering() = 0;
		virtual void setDragDropData(void* data, uint32 size) {}

	protected:
		const char* title;
		const char* dragDropTarget;

		bool windowOpen = false;
		bool windowOpenInternal = false;
	};

	struct mesh_editor_panel : asset_editor_panel
	{
		mesh_editor_panel();

		virtual void beginFrame() override;
		virtual void endFrame() override;

		void setMesh(ref<MultiMesh> m) { mesh = m; }
		void setScene(ref<EditorScene> _scene) { scene = _scene; }

		bool isHovered() const { return hovered; }

		render_camera camera;

	private:
		virtual void edit(uint32 renderWidth, uint32 renderHeight) override;
		virtual ref<dx_texture> getRendering() override;
		virtual void setDragDropData(void* data, uint32 size) override;

		ref<MultiMesh> mesh;
		ref<EditorScene> scene = nullptr;
		main_renderer renderer;

		UserInput input = {};

		camera_controller controller;

		opaque_render_pass renderPass;

		uint32 width;
		uint32 height;

		bool appFocusedLastFrame = true;
		bool hovered = false;
	};

	struct editor_panels
	{
		mesh_editor_panel meshEditor;
	};
}