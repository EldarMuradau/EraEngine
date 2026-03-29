// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#include "editor/system_calls.h"

#include <shellapi.h>

namespace era_engine
{
	void os::system_calls::showInExplorer(std::string_view path)
	{
		ShellExecuteA(NULL, "open", path_parser::makeWindowsStyle(path).c_str(), NULL, NULL, SW_SHOWNORMAL);
	}

	void os::system_calls::openFile(std::filesystem::path& path)
	{
		ShellExecuteW(0, 0, path.c_str(), 0, 0, SW_SHOWNORMAL);
	}

	void os::system_calls::editFile(std::string_view file)
	{
		ShellExecuteW(NULL, NULL, std::wstring(file.begin(), file.end()).c_str(), NULL, NULL, SW_NORMAL);
	}

	void os::system_calls::openURL(std::string_view url)
	{
		ShellExecuteA(0, 0, url.data(), 0, 0, SW_SHOW);
	}

	NODISCARD std::string os::path_parser::makeWindowsStyle(std::string_view path)
	{
		std::string result;
		result.resize(path.size());

		for (size_t i = 0; i < path.size(); ++i)
			result[i] = path[i] == '/' ? '\\' : path[i];

		return result;
	}

	NODISCARD std::string os::path_parser::makeNonWindowsStyle(std::string_view path)
	{
		std::string result;
		result.resize(path.size());

		for (size_t i = 0; i < path.size(); ++i)
			result[i] = path[i] == '\\' ? '/' : path[i];

		return result;
	}

	NODISCARD std::string os::path_parser::getContainingFolder(std::string_view path)
	{
		std::string result;

		bool extraction = false;

		for (auto it = path.rbegin(); it != path.rend(); ++it)
		{
			if (extraction)
				result += *it;

			if (!extraction && it != path.rbegin() && (*it == '\\' || *it == '/'))
				extraction = true;
		}

		std::reverse(result.begin(), result.end());

		if (!result.empty() && result.back() != '\\')
			result += '\\';

		return result;
	}

	NODISCARD std::string os::path_parser::getElementName(std::string_view path)
	{
		std::string result;

		std::string tpath = path.data();
		if (!tpath.empty() && tpath.back() == '\\')
			tpath.pop_back();

		for (auto it = tpath.rbegin(); it != tpath.rend() && *it != '\\' && *it != '/'; ++it)
			result += *it;

		std::reverse(result.begin(), result.end());

		return result;
	}

	NODISCARD std::string os::path_parser::getExtension(std::string_view path)
	{
		std::string result;

		for (auto it = path.rbegin(); it != path.rend() && *it != '.'; ++it)
			result += *it;

		std::reverse(result.begin(), result.end());

		return result;
	}

	NODISCARD std::string os::path_parser::fileTypeToString(file_type fileType)
	{
		switch (fileType)
		{
		case os::path_parser::file_type::model:		return "Model";
		case os::path_parser::file_type::texture:		return "Texture";
		case os::path_parser::file_type::shader:		return "Shader";
		case os::path_parser::file_type::material:	return "Material";
		case os::path_parser::file_type::sound:		return "Sound";
		case os::path_parser::file_type::scene:		return "Scene";
		case os::path_parser::file_type::script:		return "Script";
		case os::path_parser::file_type::font:		return "Font";
		}

		return "Unknown";
	}

	NODISCARD os::path_parser::file_type os::path_parser::getFileType(std::string_view path)
	{
		std::string ext = getExtension(path);
		std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

		if (ext == "fbx" || ext == "obj")											return os::path_parser::file_type::model;
		else if (ext == "png" || ext == "jpeg" || ext == "jpg" || ext == "tga")		return os::path_parser::file_type::texture;
		else if (ext == "glsl")														return os::path_parser::file_type::shader;
		else if (ext == "ovmat")													return os::path_parser::file_type::material;
		else if (ext == "wav" || ext == "mp3" || ext == "ogg")						return os::path_parser::file_type::sound;
		else if (ext == "ovscene")													return os::path_parser::file_type::scene;
		else if (ext == "lua")														return os::path_parser::file_type::script;
		else if (ext == "ttf")														return os::path_parser::file_type::font;

		return os::path_parser::file_type::none;
	}

}