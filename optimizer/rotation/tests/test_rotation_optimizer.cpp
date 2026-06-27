/*================================ FILE INFO =================================*/
/* Filename           : test_rotation_optimizer.cpp                           */
/*                                                                            */
/* Test implementation for rotation_optimizer.cpp                             */
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
#include "rotation.hpp"
#include "rotation_optimizer.hpp"

using namespace rotation_optimizer;

/*============================================================================*/
/*                             Public Definitions                             */
/*============================================================================*/
rotation::ControlConfig ctr_lower{};
rotation::ControlConfig ctr_upper{};
rotation::EnvironmentConfig env_lower{};
rotation::EnvironmentConfig env_upper{};

void set_local_ctr_bound_variables(void)
{
    ctr_lower.motor_speed = 55;
    ctr_lower.kp_velocity = 0;
    ctr_lower.kd_velocity = 0;
    ctr_lower.kp_angle = 0;
    ctr_lower.kd_angle = 0;
    ctr_lower.pid_scale = 16;

    ctr_upper.motor_speed = 255;
    ctr_upper.kp_velocity = 2000;
    ctr_upper.kd_velocity = 2000;
    ctr_upper.kp_angle = 100000;
    ctr_upper.kd_angle = 2000;
    ctr_upper.pid_scale = 10000;
}

void set_local_env_bound_variables(void)
{
    env_lower.dt = 0.001;
    env_lower.motor_speed_scale = 0.9;
    env_lower.motor1_variance = -0.2;
    env_lower.motor2_variance = -0.2;
    env_lower.slip_factor = 0.9;
    env_lower.wheel_circumference_scale = 0.9;
    env_lower.wheel_base_scale = 0.9;
    env_lower.rotation_angle = M_PI / 4;

    env_upper.dt = 0.001;
    env_upper.motor_speed_scale = 1.1;
    env_upper.motor1_variance = 0.2;
    env_upper.motor2_variance = 0.2;
    env_upper.slip_factor = 1.1;
    env_upper.wheel_circumference_scale = 1.1;
    env_upper.wheel_base_scale = 1.1;
    env_upper.rotation_angle = M_PI / 2;
}

void set_config_bounds(void)
{
    rotation::set_ctr_config_bounds(ctr_lower, ctr_upper);
    rotation::set_env_config_bounds(env_lower, env_upper);
}

void reset_local_and_assigned_config_bounds(void)
{
    ctr_lower = {};
    ctr_upper = {};
    env_lower = {};
    env_upper = {};
    rotation::reset_all_config_bounds();
}

/*============================================================================*/
/*                            Mock Implementations                            */
/*============================================================================*/
/* none */

/*============================================================================*/
/*                                 Test Group                                 */
/*============================================================================*/
TEST_GROUP(RotationOptimizerTests)
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
TEST(RotationOptimizerTests, RotationParetoStructureIsValid)
{
    set_local_ctr_bound_variables();
    set_local_env_bound_variables();
    set_config_bounds();

    auto result{run_rotation_staged(8, 3, 3, 2, 2)};

    CHECK_EQUAL(8, result.X.size());
    CHECK_EQUAL(8, result.F.size());

    for (size_t i{0}; i < result.X.size(); ++i) {
        CHECK_EQUAL(6, result.X.at(i).size()); /* control space */
        CHECK_EQUAL(4, result.F.at(i).size()); /* objective space */
    }
}

TEST(RotationOptimizerTests, RotationParetoHasNoNaNOrInf)
{
    set_local_ctr_bound_variables();
    set_local_env_bound_variables();
    set_config_bounds();

    auto result{run_rotation_staged(8, 3, 3, 2, 2)};

    for (const auto &f : result.F) {
        for (double v : f) {
            CHECK(std::isfinite(v));
        }
    }

    for (const auto &x : result.X) {
        for (double v : x) {
            CHECK(std::isfinite(v));
        }
    }
}

TEST(RotationOptimizerTests, RotationControlWithinBounds)
{
    set_local_ctr_bound_variables();
    set_local_env_bound_variables();
    set_config_bounds();

    auto result{run_rotation_staged(8, 3, 3, 2, 2)};

    auto bounds{rotation::get_control_bounds()};
    const auto &lb{bounds.first};
    const auto &ub{bounds.second};

    for (const auto &x : result.X) {
        for (size_t i{0}; i < x.size(); ++i) {
            CHECK(x.at(i) >= lb.at(i));
            CHECK(x.at(i) <= ub.at(i));
        }
    }
}

TEST(RotationOptimizerTests, RotationObjectivesAreInValidRanges)
{
    set_local_ctr_bound_variables();
    set_local_env_bound_variables();
    set_config_bounds();

    auto result{run_rotation_staged(8, 3, 3, 2, 2)};

    for (const auto &f : result.F) {
        double collision{f.at(0)};
        double timeout{f.at(1)};
        double angle{f.at(2)};
        double translation{f.at(3)};

        CHECK(angle >= 0.0);
        CHECK(translation >= 0.0);

        CHECK(collision >= 0.0 && collision <= 1.0);
        CHECK(timeout >= 0.0 && timeout <= 1.0);
    }
}

TEST(RotationOptimizerTests, RotationParetoSizeIsStable)
{
    set_local_ctr_bound_variables();
    set_local_env_bound_variables();
    set_config_bounds();

    auto a{run_rotation_staged(8, 3, 3, 2, 2)};
    auto b{run_rotation_staged(8, 3, 3, 2, 2)};

    CHECK_EQUAL(a.X.size(), b.X.size());
    CHECK_EQUAL(a.F.size(), b.F.size());
}

IGNORE_TEST(RotationOptimizerTests, DumpRotationPareto)
{
    set_local_ctr_bound_variables();
    set_local_env_bound_variables();
    set_config_bounds();

    /* takes ~10min */
    auto result{run_rotation_staged(64, 150, 500, 200, 50)};

    write_rotation_pareto_to_file("rotation.txt", result);
}
