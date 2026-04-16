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
#include <vector>
#include <map>
#include "simulation_common.hpp"
#include "rotation.hpp"
#include "optimizer.hpp"

using namespace optimizer;

/*============================================================================*/
/*                             Public Definitions                             */
/*============================================================================*/


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
IGNORE_TEST(OptimizerTests, RotationParetoStructureIsValid)
{
    auto result{run_rotation_pareto(8, 5)};

    CHECK_EQUAL(8, result.X.size());
    CHECK_EQUAL(8, result.F.size());

    for (size_t i{0}; i < result.X.size(); ++i) {
        CHECK_EQUAL(4, result.X.at(i).size());  /* control space */
        CHECK_EQUAL(5, result.F.at(i).size());  /* objective space */
    }
}

IGNORE_TEST(OptimizerTests, RotationParetoHasNoNaNOrInf)
{
    auto result{run_rotation_pareto(8, 5)};

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

IGNORE_TEST(OptimizerTests, RotationControlWithinBounds)
{
    auto result{run_rotation_pareto(8, 5)};

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

IGNORE_TEST(OptimizerTests, RotationObjectivesAreInValidRanges)
{
    auto result{run_rotation_pareto(8, 5)};

    for (const auto& f : result.F) {
        double angle{f.at(0)};
        double translation{f.at(1)};
        double time{f.at(2)};
        double collision{f.at(3)};
        double timeout{f.at(4)};

        CHECK(angle >= 0.0);
        CHECK(translation >= 0.0);
        CHECK(time >= 0.0);

        CHECK(collision >= 0.0 && collision <= 1.0);
        CHECK(timeout >= 0.0 && timeout <= 1.0);
    }
}

IGNORE_TEST(OptimizerTests, RotationParetoSizeIsStable)
{
    auto a{run_rotation_pareto(8, 10)};
    auto b{run_rotation_pareto(8, 10)};

    CHECK_EQUAL(a.X.size(), b.X.size());
    CHECK_EQUAL(a.F.size(), b.F.size());
}

IGNORE_TEST(OptimizerTests, DumpRotationPareto)
{
    /* takes ~5min */
    auto result{run_rotation_pareto(64, 300)};

    write_pareto_to_file("test_output.txt", result);
}
