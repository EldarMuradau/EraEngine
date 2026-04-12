// Copyright (c) 2023-present Eldar Muradov. All rights reserved.
#include <fstream>
#include <iostream>

#include "game/game_startup.h"

int main(int argc, char** argv)
{
	using namespace era_engine;

	GameStartup startup;
	startup.start(argc, argv);

	return EXIT_SUCCESS;
}