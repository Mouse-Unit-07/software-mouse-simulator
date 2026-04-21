/*================================ FILE INFO =================================*/
/* Filename           : test_front_wall_detection_optimizer.cpp               */
/*                                                                            */
/* Test implementation for front_wall_detection_optimizer.cpp                 */
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
#include "front_wall_detection.hpp"
#include "front_wall_detection_optimizer.hpp"

using namespace front_wall_detection_optimizer;

/*============================================================================*/
/*                             Public Definitions                             */
/*============================================================================*/
front_wall_detection::ControlConfig ctr_lower{};
front_wall_detection::ControlConfig ctr_upper{};
front_wall_detection::EnvironmentConfig env_lower{};
front_wall_detection::EnvironmentConfig env_upper{};

void set_local_ctr_bound_variables(void)
{
    ctr_lower.reading_threshold = 0;

    ctr_upper.reading_threshold = 1024;
}

void set_local_env_bound_variables(void)
{
    env_lower.ir_reading_scale = 0.9;
    env_lower.mouse_angle = -(M_PI / 4);
    env_lower.horizontal_position_variance = -0.9;
    env_lower.vertical_position_variance = -0.9;

    env_upper.ir_reading_scale = 1.1;
    env_upper.mouse_angle = M_PI / 4;
    env_upper.horizontal_position_variance = 0.9;
    env_upper.vertical_position_variance = 0.9;
}

void set_config_bounds(void)
{
    front_wall_detection::set_ctr_config_bounds(ctr_lower, ctr_upper);
    front_wall_detection::set_env_config_bounds(env_lower, env_upper);
}

void reset_local_and_assigned_config_bounds(void)
{
    ctr_lower = {};
    ctr_upper = {};
    env_lower = {};
    env_upper = {};
    front_wall_detection::reset_all_config_bounds();
}

/*============================================================================*/
/*                            Mock Implementations                            */
/*============================================================================*/
/* none */

/*============================================================================*/
/*                                 Test Group                                 */
/*============================================================================*/
TEST_GROUP(FrontWallDetectionOptimizerTests)
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
TEST(FrontWallDetectionOptimizerTests, ParetoStructureIsValid)
{
    set_local_ctr_bound_variables();
    set_local_env_bound_variables();
    set_config_bounds();

    auto result{run_front_wall_detection_staged(8, 3, 2)};

    CHECK_EQUAL(8, result.X.size());
    CHECK_EQUAL(8, result.F.size());

    for (size_t i{0}; i < result.X.size(); ++i) {
        CHECK_EQUAL(1, result.X.at(i).size()); /* control space */
        CHECK_EQUAL(2, result.F.at(i).size()); /* objective space */
    }
}

TEST(FrontWallDetectionOptimizerTests, ParetoHasNoNaNOrInf)
{
    set_local_ctr_bound_variables();
    set_local_env_bound_variables();
    set_config_bounds();

    auto result{run_front_wall_detection_staged(8, 3, 2)};

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

TEST(FrontWallDetectionOptimizerTests, ControlWithinBounds)
{
    set_local_ctr_bound_variables();
    set_local_env_bound_variables();
    set_config_bounds();

    auto result{run_front_wall_detection_staged(8, 3, 2)};

    auto bounds{front_wall_detection::get_control_bounds()};
    const auto& lb{bounds.first};
    const auto& ub{bounds.second};

    for (const auto& x : result.X) {
        for (size_t i{0}; i < x.size(); ++i) {
            CHECK(x.at(i) >= lb.at(i));
            CHECK(x.at(i) <= ub.at(i));
        }
    }
}

TEST(FrontWallDetectionOptimizerTests, ObjectivesAreInValidRanges)
{
    set_local_ctr_bound_variables();
    set_local_env_bound_variables();
    set_config_bounds();

    auto result{run_front_wall_detection_staged(8, 3, 2)};

    for (const auto& f : result.F) {
        double identified_absent_wall{-f.at(0)};
        double identified_present_wall{-f.at(1)};

        CHECK((identified_absent_wall >= 0.0) && (identified_absent_wall <= 1.0));
        CHECK((identified_present_wall >= 0.0) && (identified_present_wall <= 1.0));
    }
}

TEST(FrontWallDetectionOptimizerTests, ParetoSizeIsStable)
{
    set_local_ctr_bound_variables();
    set_local_env_bound_variables();
    set_config_bounds();

    auto a{run_front_wall_detection_staged(8, 3, 2)};
    auto b{run_front_wall_detection_staged(8, 3, 2)};

    CHECK_EQUAL(a.X.size(), b.X.size());
    CHECK_EQUAL(a.F.size(), b.F.size());
}

IGNORE_TEST(FrontWallDetectionOptimizerTests, DumpPareto)
{
    set_local_ctr_bound_variables();
    set_local_env_bound_variables();
    set_config_bounds();

    /* takes ~11min */
    auto result{run_front_wall_detection_staged(64, 500, 1000)};

    write_pareto_to_file("fwd.txt", result);
}
