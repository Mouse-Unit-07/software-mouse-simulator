/*================================ FILE INFO =================================*/
/* Filename           : test_visualizer.cpp                                   */
/*                                                                            */
/* Test implementation for visualizer.cpp                                     */
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
#include "rotation.hpp"
#include "move_forward.hpp"
#include "optimizer.hpp"

using namespace optimizer;

/*============================================================================*/
/*                             Public Definitions                             */
/*============================================================================*/
move_forward::ControlConfig ctr_lower{};
move_forward::ControlConfig ctr_upper{};
move_forward::EnvironmentConfig env_lower{};
move_forward::EnvironmentConfig env_upper{};

void set_local_ctr_bound_variables(void)
{
    ctr_lower.single_wall_target = 0;
    ctr_lower.motor_speed = 140;
    ctr_lower.kp_velocity = 0;
    ctr_lower.kd_velocity = 0;
    ctr_lower.kp_angle = 0;
    ctr_lower.kd_angle = 0;
    ctr_lower.pid_scale = 16;
    ctr_lower.kp_ir = 0;
    ctr_lower.kd_ir = 0;

    ctr_upper.single_wall_target = 1024;
    ctr_upper.motor_speed = 255;
    ctr_upper.kp_velocity = 2000;
    ctr_upper.kd_velocity = 2000;
    ctr_upper.kp_angle = 2000;
    ctr_upper.kd_angle = 2000;
    ctr_upper.pid_scale = 512;
    ctr_upper.kp_ir = 2000;
    ctr_upper.kd_ir = 2000;
}

void set_local_env_bound_variables(void)
{
    env_lower.dt = 0.005;
    env_lower.motor_speed_scale = 0.9;
    env_lower.motor1_variance = -0.2;
    env_lower.motor2_variance = -0.2;
    env_lower.slip_factor = 0.9;
    env_lower.wheel_circumference_scale = 0.9;
    env_lower.wheel_base_scale = 0.9;
    env_lower.maze_size_scale = 0.9;
    env_lower.ir_reading_scale = 0.9;
    env_lower.mouse_angle = -(M_PI / 4);
    env_lower.horizontal_position_variance = -0.5;
    env_lower.vertical_position_variance = -0.5;

    env_upper.dt = 0.01;
    env_upper.motor_speed_scale = 1.1;
    env_upper.motor1_variance = 0.2;
    env_upper.motor2_variance = 0.2;
    env_upper.slip_factor = 1.1;
    env_upper.wheel_circumference_scale = 1.1;
    env_upper.wheel_base_scale = 1.1;
    env_upper.maze_size_scale = 1.1;
    env_upper.ir_reading_scale = 1.1;
    env_upper.mouse_angle = M_PI / 4;
    env_upper.horizontal_position_variance = 0.5;
    env_upper.vertical_position_variance = 0.5;
}

void set_config_bounds(void)
{
    move_forward::set_ctr_config_bounds(ctr_lower, ctr_upper);
    move_forward::set_env_config_bounds(env_lower, env_upper);
}

void reset_local_and_assigned_config_bounds(void)
{
    ctr_lower = {};
    ctr_upper = {};
    env_lower = {};
    env_upper = {};
    move_forward::reset_all_config_bounds();
}

/*============================================================================*/
/*                            Mock Implementations                            */
/*============================================================================*/
/* none */

/*============================================================================*/
/*                                 Test Group                                 */
/*============================================================================*/
TEST_GROUP(OptimizerTests)
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
TEST(OptimizerTests, RotationParetoStructureIsValid)
{
    auto result{run_rotation_pareto(8, 5, 10)};

    CHECK_EQUAL(8, result.X.size());
    CHECK_EQUAL(8, result.F.size());

    for (size_t i{0}; i < result.X.size(); ++i) {
        CHECK_EQUAL(6, result.X.at(i).size()); /* control space */
        CHECK_EQUAL(4, result.F.at(i).size()); /* objective space */
    }
}

TEST(OptimizerTests, RotationParetoHasNoNaNOrInf)
{
    auto result{run_rotation_pareto(8, 5, 10)};

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

TEST(OptimizerTests, RotationControlWithinBounds)
{
    auto result{run_rotation_pareto(8, 5, 10)};

    auto bounds{rotation::get_control_bounds()};
    const auto& lb{bounds.first};
    const auto& ub{bounds.second};

    for (const auto& x : result.X) {
        for (size_t i{0}; i < x.size(); ++i) {
            CHECK(x.at(i) >= lb.at(i));
            CHECK(x.at(i) <= ub.at(i));
        }
    }
}

TEST(OptimizerTests, RotationObjectivesAreInValidRanges)
{
    auto result{run_rotation_pareto(8, 5, 10)};

    for (const auto& f : result.F) {
        double angle{f.at(0)};
        double translation{f.at(1)};
        double collision{f.at(2)};
        double timeout{f.at(3)};

        CHECK(angle >= 0.0);
        CHECK(translation >= 0.0);

        CHECK(collision >= 0.0 && collision <= 1.0);
        CHECK(timeout >= 0.0 && timeout <= 1.0);
    }
}

TEST(OptimizerTests, RotationParetoSizeIsStable)
{
    auto a{run_rotation_pareto(8, 10, 10)};
    auto b{run_rotation_pareto(8, 10, 10)};

    CHECK_EQUAL(a.X.size(), b.X.size());
    CHECK_EQUAL(a.F.size(), b.F.size());
}

IGNORE_TEST(OptimizerTests, DumpRotationPareto)
{
    /* takes ~5min */
    auto result{run_rotation_pareto(64, 300, 1000)};

    write_rotation_pareto_to_file("rotation_test_output.txt", result);
}

TEST(OptimizerTests, MoveForwardParetoStructureIsValid)
{
    set_local_ctr_bound_variables();
    set_local_env_bound_variables();
    set_config_bounds();

    for (move_forward::WallMode m : move_forward::WALL_MODES) {
        auto result{run_move_forward_staged(8, 3, 3, 2, 2, m)};

        CHECK_EQUAL(8, result.X.size());
        CHECK_EQUAL(8, result.F.size());

        for (size_t i{0}; i < result.X.size(); ++i) {
            CHECK_EQUAL(9, result.X.at(i).size()); /* control space */
            CHECK_EQUAL(5, result.F.at(i).size()); /* objective space */
        }
    }
}

TEST(OptimizerTests, MoveForwardParetoHasNoNaNOrInf)
{
    set_local_ctr_bound_variables();
    set_local_env_bound_variables();
    set_config_bounds();

    for (move_forward::WallMode m : move_forward::WALL_MODES) {
        auto result{run_move_forward_staged(8, 3, 3, 2, 2, m)};

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
}

TEST(OptimizerTests, MoveForwardControlWithinBounds)
{
    set_local_ctr_bound_variables();
    set_local_env_bound_variables();
    set_config_bounds();

    for (move_forward::WallMode m : move_forward::WALL_MODES) {
        auto result{run_move_forward_staged(8, 3, 3, 2, 2, m)};

        auto bounds{move_forward::get_control_bounds()};
        const auto& lb{bounds.first};
        const auto& ub{bounds.second};

        for (const auto& x : result.X) {
            for (size_t i{0}; i < x.size(); ++i) {
                CHECK(x.at(i) >= lb.at(i));
                CHECK(x.at(i) <= ub.at(i));
            }
        }
    }
}

TEST(OptimizerTests, MoveForwardObjectivesAreInValidRanges)
{
    set_local_ctr_bound_variables();
    set_local_env_bound_variables();
    set_config_bounds();

    for (move_forward::WallMode m : move_forward::WALL_MODES) {
        auto result{run_move_forward_staged(8, 3, 3, 2, 2, m)};

        for (const auto& f : result.F) {
            double collision{f.at(0)};
            double horizontal_translation{f.at(1)};
            double timeout{f.at(2)};
            double vertical_translation{f.at(3)};
            double angle{f.at(4)};

            CHECK(angle >= 0.0);
            CHECK(horizontal_translation >= 0.0);
            CHECK(vertical_translation >= 0.0);

            CHECK(collision >= 0.0 && collision <= 1.0);
            CHECK(timeout >= 0.0 && timeout <= 1.0);
        }
    }
}

TEST(OptimizerTests, MoveForwardParetoSizeIsStable)
{
    set_local_ctr_bound_variables();
    set_local_env_bound_variables();
    set_config_bounds();

    for (move_forward::WallMode m : move_forward::WALL_MODES) {
        auto a{run_move_forward_staged(8, 3, 3, 2, 2, m)};
        auto b{run_move_forward_staged(8, 3, 3, 2, 2, m)};

        CHECK_EQUAL(a.X.size(), b.X.size());
        CHECK_EQUAL(a.F.size(), b.F.size());
    }
}

IGNORE_TEST(OptimizerTests, DumpMoveForwardParetoNoWalls)
{
    set_local_ctr_bound_variables();
    set_local_env_bound_variables();
    env_lower.mouse_angle = 0.0;
    env_upper.mouse_angle = 0.0;

    set_config_bounds();

    auto no_walls{run_move_forward_staged(64, 150, 500, 200, 50, move_forward::WallMode::NO_WALLS)};
    write_move_forward_pareto_to_file("mf_no_walls.txt", no_walls);
}

IGNORE_TEST(OptimizerTests, DumpMoveForwardParetoOneWall)
{
    set_local_ctr_bound_variables();
    set_local_env_bound_variables();
    set_config_bounds();

    auto one_wall{
        run_move_forward_staged(64, 150, 500, 200, 50, move_forward::WallMode::LEFT_WALL_ONLY)};
    write_move_forward_pareto_to_file("mf_one_wall.txt", one_wall);
}

IGNORE_TEST(OptimizerTests, DumpMoveForwardParetoBothWalls)
{
    set_local_ctr_bound_variables();
    set_local_env_bound_variables();
    set_config_bounds();

    /* takes ~11min */
    auto both_walls{
        run_move_forward_staged(64, 150, 500, 200, 50, move_forward::WallMode::BOTH_WALLS)};
    write_move_forward_pareto_to_file("mf_both_walls.txt", both_walls);
}
