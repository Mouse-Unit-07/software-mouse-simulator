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
/* none */

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

    }

    void teardown() override
    {

    }
};

/*============================================================================*/
/*                                    Tests                                   */
/*============================================================================*/
TEST(RotationOptimizerTests, RotationParetoStructureIsValid)
{
    auto result{run_rotation_pareto(8, 5, 10)};

    CHECK_EQUAL(8, result.X.size());
    CHECK_EQUAL(8, result.F.size());

    for (size_t i{0}; i < result.X.size(); ++i) {
        CHECK_EQUAL(6, result.X.at(i).size()); /* control space */
        CHECK_EQUAL(4, result.F.at(i).size()); /* objective space */
    }
}

TEST(RotationOptimizerTests, RotationParetoHasNoNaNOrInf)
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

TEST(RotationOptimizerTests, RotationControlWithinBounds)
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

TEST(RotationOptimizerTests, RotationObjectivesAreInValidRanges)
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

TEST(RotationOptimizerTests, RotationParetoSizeIsStable)
{
    auto a{run_rotation_pareto(8, 10, 10)};
    auto b{run_rotation_pareto(8, 10, 10)};

    CHECK_EQUAL(a.X.size(), b.X.size());
    CHECK_EQUAL(a.F.size(), b.F.size());
}

IGNORE_TEST(RotationOptimizerTests, DumpRotationPareto)
{
    /* takes ~5min */
    auto result{run_rotation_pareto(64, 300, 1000)};

    write_rotation_pareto_to_file("rotation_test_output.txt", result);
}
