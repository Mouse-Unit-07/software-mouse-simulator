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
/* none */

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

    }

    void teardown() override
    {

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
    for (move_forward::WallMode m : move_forward::WALL_MODES) {
        auto result{run_move_forward_pareto(8, 5, 10, m)};

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
    for (move_forward::WallMode m : move_forward::WALL_MODES) {
        auto result{run_move_forward_pareto(8, 5, 10, m)};

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
    for (move_forward::WallMode m : move_forward::WALL_MODES) {
        auto result{run_move_forward_pareto(8, 5, 10, m)};

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
    for (move_forward::WallMode m : move_forward::WALL_MODES) {
        auto result{run_move_forward_pareto(8, 5, 10, m)};

        for (const auto& f : result.F) {
            double angle{f.at(0)};
            double horizontal_translation{f.at(1)};
            double vertical_translation{f.at(2)};
            double collision{f.at(3)};
            double timeout{f.at(4)};

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
    for (move_forward::WallMode m : move_forward::WALL_MODES) {
        auto a{run_move_forward_pareto(8, 10, 10, m)};
        auto b{run_move_forward_pareto(8, 10, 10, m)};

        CHECK_EQUAL(a.X.size(), b.X.size());
        CHECK_EQUAL(a.F.size(), b.F.size());
    }
}

IGNORE_TEST(OptimizerTests, DumpMoveForwardPareto)
{
    /* takes ~5min */
    auto no_walls{run_move_forward_pareto(64, 300, 50, move_forward::WallMode::NO_WALLS)};
    write_move_forward_pareto_to_file("move_forward_test_no_walls.txt", no_walls);

    auto one_wall{run_move_forward_pareto(64, 300, 50, move_forward::WallMode::LEFT_WALL_ONLY)};
    write_move_forward_pareto_to_file("move_forward_test_one_wall.txt", one_wall);

    auto both_walls{run_move_forward_pareto(64, 300, 50, move_forward::WallMode::BOTH_WALLS)};
    write_move_forward_pareto_to_file("move_forward_test_both_walls.txt", both_walls);
}
