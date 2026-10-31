// Copyright (c) 2023-present Eldar Muradov. All rights reserved.

#include <gtest/gtest.h>

#include <core/job_system.h>

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    era_engine::initialize_job_system();

    const int result = RUN_ALL_TESTS();

    era_engine::shutdown_job_system();

    return result;
}