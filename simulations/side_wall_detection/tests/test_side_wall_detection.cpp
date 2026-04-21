/*================================ FILE INFO =================================*/
/* Filename           : test_side_wall_detection.cpp                          */
/*                                                                            */
/* Test implementation for side_wall_detection.cpp                            */
/*                                                                            */
/*============================================================================*/

/*============================================================================*/
/*                               Include Files                                */
/*============================================================================*/
#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>
#include <cmath>
#include <cstdint>
#include <map>
#include <string>
#include <vector>
#include "simulation_common.hpp"
#include "side_wall_detection.hpp"

using namespace side_wall_detection;

/*============================================================================*/
/*                             Public Definitions                             */
/*============================================================================*/
constexpr double FLOAT_TOLERANCE{1e-6};

Config create_no_variance_config(void)
{
    Config cfg{};
    cfg.maze_size_scale = 1.0;
    cfg.ir_reading_scale = 1.0;
    cfg.mouse_angle = 0.0;
    cfg.horizontal_position_variance = 0.0;
    cfg.vertical_position_variance = 0.0;
    cfg.total_steps = 100;
    cfg.reading_threshold = 80u; /* arbitrary threshold */

    return cfg;
}

bool are_results_equivalent(const Result& r1, const Result& r2)
{
    if (r1.wall_absent_at_step.size() != r2.wall_absent_at_step.size()) {
        return false;
    }
    if (r1.wall_present_at_step.size() != r2.wall_present_at_step.size()) {
        return false;
    }

    for (size_t i{0}; i < r1.wall_absent_at_step.size(); ++i) {
        if (r1.wall_absent_at_step.at(i) != r2.wall_absent_at_step.at(i)) {
            return false;
        }
        if (r1.wall_present_at_step.at(i) != r2.wall_present_at_step.at(i)) {
            return false;
        }
    }

    return true;
}

/*============================================================================*/
/*                            Mock Implementations                            */
/*============================================================================*/
/* none */

/*============================================================================*/
/*                                 Test Group                                 */
/*============================================================================*/
TEST_GROUP(SideWallDetectionTests)
{
    void setup() override
    {
        disable_visualization();
    }

    void teardown() override
    {
        disable_visualization();
    }
};

/*============================================================================*/
/*                                    Tests                                   */
/*============================================================================*/
TEST(SideWallDetectionTests, SimulationHandlesZeroSteps)
{
    Config cfg{create_no_variance_config()};
    cfg.total_steps = 0;

    auto result{run_simulation(cfg)};

    CHECK_EQUAL(0, result.wall_absent_at_step.size());
    CHECK_EQUAL(0, result.wall_present_at_step.size());
}

TEST(SideWallDetectionTests, WallAbsentAllFalseWhenThresholdIsZero)
{
    Config cfg{create_no_variance_config()};
    cfg.reading_threshold = 0u; /* nothing should pass */

    auto result{run_simulation(cfg)};

    for (bool v : result.wall_absent_at_step) {
        CHECK_FALSE(v);
    }
}

TEST(SideWallDetectionTests, WallPresentAllTrueWhenThresholdIsZero)
{
    Config cfg{create_no_variance_config()};
    cfg.reading_threshold = 0u; /* everything should pass */

    auto result{run_simulation(cfg)};

    for (bool v : result.wall_present_at_step) {
        CHECK(v);
    }
}

TEST(SideWallDetectionTests, IdenticalConfigsProduceIdenticalResults)
{
    Config cfg{create_no_variance_config()};

    auto r1{run_simulation(cfg)};
    auto r2{run_simulation(cfg)};

    CHECK(are_results_equivalent(r1, r2));
}

TEST(SideWallDetectionTests, MazeSizeScaleChangesResults)
{
    Config cfg1{create_no_variance_config()};
    Config cfg2{cfg1};
    cfg2.maze_size_scale = 10.0;

    auto r1{run_simulation(cfg1)};
    auto r2{run_simulation(cfg2)};

    CHECK(!are_results_equivalent(r1, r2));
}

TEST(SideWallDetectionTests, IrReadingScaleChangesResults)
{
    Config cfg1{create_no_variance_config()};
    Config cfg2{cfg1};
    cfg2.ir_reading_scale = 0.5;

    auto r1{run_simulation(cfg1)};
    auto r2{run_simulation(cfg2)};

    CHECK(!are_results_equivalent(r1, r2));
}

TEST(SideWallDetectionTests, ZeroIrReadingScaleCollapsesToAllTrueWhenThresholdIsMax)
{
    Config cfg{create_no_variance_config()};
    cfg.reading_threshold = 1024u;

    auto result{run_simulation(cfg)};

    for (bool v : result.wall_absent_at_step) {
        CHECK(v);
    }
}

TEST(SideWallDetectionTests, MouseAngleChangesResults)
{
    Config cfg1{create_no_variance_config()};
    Config cfg2{cfg1};
    cfg2.mouse_angle = M_PI / 2;

    auto r1{run_simulation(cfg1)};
    auto r2{run_simulation(cfg2)};

    CHECK(!are_results_equivalent(r1, r2));
}

TEST(SideWallDetectionTests, HorizontalVarianceChangesResults)
{
    Config cfg1{create_no_variance_config()};
    Config cfg2{cfg1};
    cfg2.horizontal_position_variance = -0.9;

    auto r1{run_simulation(cfg1)};
    auto r2{run_simulation(cfg2)};

    CHECK(!are_results_equivalent(r1, r2));
}

TEST(SideWallDetectionTests, VerticalVarianceChangesResults)
{
    Config cfg1{create_no_variance_config()};
    Config cfg2{cfg1};
    cfg2.vertical_position_variance = -0.9;

    auto r1{run_simulation(cfg1)};
    auto r2{run_simulation(cfg2)};

    CHECK(!are_results_equivalent(r1, r2));
}

IGNORE_TEST(SideWallDetectionTests, VisualizationDoesNotAffectResults)
{
    Config cfg{create_no_variance_config()};

    disable_visualization();
    auto r1{run_simulation(cfg)};

    enable_visualization();
    auto r2{run_simulation(cfg)};

    CHECK(are_results_equivalent(r1, r2));
}
