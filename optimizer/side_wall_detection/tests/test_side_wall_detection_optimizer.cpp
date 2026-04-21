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

    ctr_upper.reading_threshold = 1024;
}

void set_local_env_bound_variables(void)
{
    env_lower.maze_size_scale = 0.9;
    env_lower.ir_reading_scale = 0.9;
    env_lower.mouse_angle = -(M_PI / 6);
    env_lower.horizontal_position_variance = -0.5;
    env_lower.vertical_position_variance = -0.5;
    env_lower.total_steps = 100;

    env_upper.maze_size_scale = 1.1;
    env_upper.ir_reading_scale = 1.1;
    env_upper.mouse_angle = M_PI / 6;
    env_upper.horizontal_position_variance = 0.5;
    env_upper.vertical_position_variance = 0.5;
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
        CHECK_EQUAL(1, result.X.at(i).size()); /* control space */
        CHECK_EQUAL(4, result.F.at(i).size()); /* objective space */
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
        double w30 = -f.at(0);
        double w60 = -f.at(1);
        double w90 = -f.at(2);
        double w100 = -f.at(3);

        CHECK((w30 >= 0.0) && (w30 <= 1.0));
        CHECK((w60 >= 0.0) && (w60 <= 1.0));
        CHECK((w90 >= 0.0) && (w90 <= 1.0));
        CHECK((w100 >= 0.0) && (w100 <= 1.0));
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

    /* takes ~24min */
    auto result{run_side_wall_detection_filter(64, 30, 1000)};

    write_pareto_to_file("swd.txt", result);
}
