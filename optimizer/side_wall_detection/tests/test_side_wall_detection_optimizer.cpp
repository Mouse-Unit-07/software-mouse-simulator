/*================================ FILE INFO =================================*/
/* Filename           : test_side_wall_detection_optimizer.cpp                */
/*                                                                            */
/* Test implementation for side_wall_detection_optimizer.cpp                  */
/*                                                                            */
/*============================================================================*/

/*============================================================================*/
/*                               Include Files                                */
/*============================================================================*/
#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <map>
#include <vector>
#include "simulation_common.hpp"
#include "optimizer_common.hpp"
#include "side_wall_detection.hpp"
#include "side_wall_detection_optimizer.hpp"

using namespace side_wall_detection_optimizer;

/*============================================================================*/
/*                             Public Definitions                             */
/*============================================================================*/
side_wall_detection::ControlConfig ctr_lower{};
side_wall_detection::ControlConfig ctr_upper{};
side_wall_detection::EnvironmentConfig env_lower{};
side_wall_detection::EnvironmentConfig env_upper{};

void set_local_ctr_bound_variables(void)
{
    ctr_lower.reading_threshold = 0;
    ctr_lower.reading_start_offset = 0.0;

    ctr_upper.reading_threshold = 500;
    ctr_upper.reading_start_offset = 0.9;
}

void set_local_env_bound_variables(void)
{
    env_lower.maze_size_scale = 0.9;
    env_lower.ir_reading_scale = 0.9;
    env_lower.mouse_angle = -(M_PI / 4);
    env_lower.horizontal_position_variance = -0.9;
    env_lower.vertical_position_variance = -0.9;
    env_lower.total_steps = 100;

    env_upper.maze_size_scale = 1.1;
    env_upper.ir_reading_scale = 1.1;
    env_upper.mouse_angle = M_PI / 4;
    env_upper.horizontal_position_variance = 0.9;
    env_upper.vertical_position_variance = 0.9;
    env_upper.total_steps = 100;
}

void set_config_bounds(void)
{
    side_wall_detection::set_ctr_config_bounds(ctr_lower, ctr_upper);
    side_wall_detection::set_env_config_bounds(env_lower, env_upper);
}

void reset_local_and_assigned_config_bounds(void)
{
    ctr_lower = {};
    ctr_upper = {};
    env_lower = {};
    env_upper = {};
    side_wall_detection::reset_all_config_bounds();
}

/*============================================================================*/
/*                            Mock Implementations                            */
/*============================================================================*/
/* none */

/*============================================================================*/
/*                                 Test Group                                 */
/*============================================================================*/
TEST_GROUP(SideWallDetectionOptimizerTests)
{
    void setup() override
    {
        reset_local_and_assigned_config_bounds();
    }

    void teardown() override
    {
        reset_local_and_assigned_config_bounds();
    }
};

/*============================================================================*/
/*                                    Tests                                   */
/*============================================================================*/
TEST(SideWallDetectionOptimizerTests, ParetoStructureIsValid)
{
    set_local_ctr_bound_variables();
    set_local_env_bound_variables();
    set_config_bounds();

    auto result{run_side_wall_detection_filter(8, 3, 2)};

    CHECK_EQUAL(8, result.X.size());
    CHECK_EQUAL(8, result.F.size());

    for (size_t i{0}; i < result.X.size(); ++i) {
        CHECK_EQUAL(2, result.X.at(i).size()); /* control space */
        CHECK_EQUAL(2, result.F.at(i).size()); /* objective space */
    }
}

TEST(SideWallDetectionOptimizerTests, ParetoHasNoNaNOrInf)
{
    set_local_ctr_bound_variables();
    set_local_env_bound_variables();
    set_config_bounds();

    auto result{run_side_wall_detection_filter(8, 3, 2)};

    for (const auto& f : result.F) {
        for (double v : f) {
            CHECK(std::isfinite(v));
        }
    }

    for (const auto& x : result.X) {
        for (double v : x) {
            CHECK(std::isfinite(v));
        }
    }
}

TEST(SideWallDetectionOptimizerTests, ControlWithinBounds)
{
    set_local_ctr_bound_variables();
    set_local_env_bound_variables();
    set_config_bounds();

    auto result{run_side_wall_detection_filter(8, 3, 2)};

    auto bounds{side_wall_detection::get_control_bounds()};
    const auto& lb{bounds.first};
    const auto& ub{bounds.second};

    for (const auto& x : result.X) {
        for (size_t i{0}; i < x.size(); ++i) {
            CHECK(x.at(i) >= lb.at(i));
            CHECK(x.at(i) <= ub.at(i));
        }
    }
}

TEST(SideWallDetectionOptimizerTests, ObjectivesAreInValidRanges)
{
    set_local_ctr_bound_variables();
    set_local_env_bound_variables();
    set_config_bounds();

    auto result{run_side_wall_detection_filter(8, 3, 2)};

    for (const auto& f : result.F) {
        double combined{-f.at(0)};
        double offset{f.at(1)};

        CHECK((combined >= 0.0) && (combined <= 1.0));
        CHECK((offset >= 0.0) && (offset <= 0.9));
    }
}

TEST(SideWallDetectionOptimizerTests, ParetoSizeIsStable)
{
    set_local_ctr_bound_variables();
    set_local_env_bound_variables();
    set_config_bounds();

    auto a{run_side_wall_detection_filter(8, 3, 2)};
    auto b{run_side_wall_detection_filter(8, 3, 2)};

    CHECK_EQUAL(a.X.size(), b.X.size());
    CHECK_EQUAL(a.F.size(), b.F.size());
}

IGNORE_TEST(SideWallDetectionOptimizerTests, DumpPareto)
{
    set_local_ctr_bound_variables();
    set_local_env_bound_variables();
    set_config_bounds();

    /* takes ~5min */
    auto result{run_side_wall_detection_filter(64, 50, 1000)};

    write_pareto_to_file("swd.txt", result);
}
