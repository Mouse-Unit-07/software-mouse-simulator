/*================================ FILE INFO =================================*/
/* Filename           : test_wall_detection.cpp                               */
/*                                                                            */
/* Test implementation for wall_detection.cpp                                 */
/*                                                                            */
/*============================================================================*/

/*============================================================================*/
/*                               Include Files                                */
/*============================================================================*/
extern "C"
{

#include <stdint.h>
#include <math.h>
#include "mock_device_drivers.h"
#include "infrared_sensor.h"

}

#include <vector>
#include <string>
#include <optional>
#include "point.hpp"
#include "ray.hpp"
#include "rectangular_hitbox.hpp"
#include "mouse.hpp"
#include "maze.hpp"
#include "wall_detection.hpp"

#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>

using namespace wall_detection;

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
TEST_GROUP(WallDetectionTests)
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
TEST(WallDetectionTests, BuildConfigMapsValuesCorrectly)
{
    std::vector<double> v {1, 2, 3, 4, 5, 6, 7};

    auto cfg {build_config(v)};

    CHECK_EQUAL(1, cfg.maze_size_scale);
    CHECK_EQUAL(2, cfg.ir_reading_scale);
    CHECK_EQUAL(3, cfg.mouse_angle);
    CHECK_EQUAL(4, cfg.horizontal_position_variance);
    CHECK_EQUAL(5, cfg.vertical_position_variance);
    CHECK_EQUAL(6, cfg.total_steps);
    CHECK_EQUAL(7, cfg.reading_threshold);
}

TEST(WallDetectionTests, SimulationHandlesZeroSteps)
{
    Config cfg{};
    cfg.total_steps = 0;

    auto result {run_simulation(cfg)};

    CHECK_EQUAL(0, result.correct_detection_at_step.size());
}

TEST(WallDetectionTests, AllFalseWhenThresholdIsZero)
{
    Config cfg{};
    cfg.maze_size_scale = 1.0;
    cfg.ir_reading_scale = 1.0;
    cfg.mouse_angle = 0.0;
    cfg.horizontal_position_variance = 0.0;
    cfg.vertical_position_variance = 0.0;
    cfg.total_steps = 100;
    cfg.reading_threshold = 0u;  /* nothing should pass */

    auto result {run_simulation(cfg)};

    for (bool v : result.correct_detection_at_step) {
        CHECK_FALSE(v);
    }
}

TEST(WallDetectionTests, IdenticalConfigsProduceIdenticalResults)
{
    Config cfg{};
    cfg.maze_size_scale = 1.0;
    cfg.ir_reading_scale = 1.0;
    cfg.mouse_angle = 0.0;
    cfg.horizontal_position_variance = 0.0;
    cfg.vertical_position_variance = 0.0;
    cfg.total_steps = 100;
    cfg.reading_threshold = 750u;

    auto r1 {run_simulation(cfg)};
    auto r2 {run_simulation(cfg)};

    CHECK_EQUAL(r1.correct_detection_at_step.size(),
                r2.correct_detection_at_step.size());

    for (size_t i {0}; i < r1.correct_detection_at_step.size(); ++i) {
        CHECK_EQUAL(r1.correct_detection_at_step[i],
                    r2.correct_detection_at_step[i]);
    }
}

TEST(WallDetectionTests, MazeSizeScaleChangesResults)
{
    Config cfg1{};
    cfg1.maze_size_scale = 1.0;
    cfg1.ir_reading_scale = 1.0;
    cfg1.mouse_angle = 0;
    cfg1.horizontal_position_variance = 0.0;
    cfg1.vertical_position_variance = 0.0;
    cfg1.total_steps = 100;
    cfg1.reading_threshold = 750u;

    Config cfg2 {cfg1};
    cfg2.maze_size_scale = 10.0;

    auto r1 {run_simulation(cfg1)};
    auto r2 {run_simulation(cfg2)};

    bool diff_found {false};

    for (size_t i {0}; i < r1.correct_detection_at_step.size(); ++i) {
        if (r1.correct_detection_at_step[i] != r2.correct_detection_at_step[i]) {
            diff_found = true;
            break;
        }
    }

    CHECK(diff_found);
}

TEST(WallDetectionTests, IrReadingScaleChangesResults)
{
    Config cfg1{};
    cfg1.maze_size_scale = 1.0;
    cfg1.ir_reading_scale = 1.0;
    cfg1.mouse_angle = 0;
    cfg1.horizontal_position_variance = 0.0;
    cfg1.vertical_position_variance = 0.0;
    cfg1.total_steps = 100;
    cfg1.reading_threshold = 750u;

    Config cfg2 {cfg1};
    cfg2.ir_reading_scale = 0.5;

    auto r1 {run_simulation(cfg1)};
    auto r2 {run_simulation(cfg2)};

    bool diff_found {false};

    for (size_t i {0}; i < r1.correct_detection_at_step.size(); ++i) {
        if (r1.correct_detection_at_step[i] != r2.correct_detection_at_step[i]) {
            diff_found = true;
            break;
        }
    }

    CHECK(diff_found);
}

TEST(WallDetectionTests, ZeroIrReadingScaleCollapsesToAllTrueWhenThresholdIsMax)
{
    Config cfg{};
    cfg.maze_size_scale = 1.0;
    cfg.ir_reading_scale = 0.0;
    cfg.mouse_angle = 0.0;
    cfg.horizontal_position_variance = 0.0;
    cfg.vertical_position_variance = 0.0;
    cfg.total_steps = 100;
    cfg.reading_threshold = 1024u;

    auto result {run_simulation(cfg)};

    for (bool v : result.correct_detection_at_step) {
        CHECK(v);
    }
}

TEST(WallDetectionTests, MouseAngleChangesResults)
{
    Config cfg1{};
    cfg1.maze_size_scale = 1.0;
    cfg1.ir_reading_scale = 1.0;
    cfg1.mouse_angle = 0.0;
    cfg1.horizontal_position_variance = 0.0;
    cfg1.vertical_position_variance = 0.0;
    cfg1.total_steps = 100;
    cfg1.reading_threshold = 750u;

    Config cfg2 {cfg1};
    cfg2.mouse_angle = M_PI / 2;

    auto r1 {run_simulation(cfg1)};
    auto r2 {run_simulation(cfg2)};

    bool diff_found {false};

    for (size_t i {0}; i < r1.correct_detection_at_step.size(); ++i) {
        if (r1.correct_detection_at_step[i] != r2.correct_detection_at_step[i]) {
            diff_found = true;
            break;
        }
    }

    CHECK(diff_found);
}

TEST(WallDetectionTests, HorizontalVarianceChangesResults)
{
    Config cfg1{};
    cfg1.maze_size_scale = 1.0;
    cfg1.ir_reading_scale = 1.0;
    cfg1.mouse_angle = M_PI / 8;
    cfg1.horizontal_position_variance = 1.0;
    cfg1.vertical_position_variance = 0.0;
    cfg1.total_steps = 100;
    cfg1.reading_threshold = 900u;

    Config cfg2 {cfg1};
    cfg2.horizontal_position_variance = -0.9;

    auto r1 {run_simulation(cfg1)};
    auto r2 {run_simulation(cfg2)};

    bool diff_found {false};

    for (size_t i {0}; i < r1.correct_detection_at_step.size(); ++i) {
        if (r1.correct_detection_at_step[i] != r2.correct_detection_at_step[i]) {
            diff_found = true;
            break;
        }
    }

    CHECK(diff_found);
}

TEST(WallDetectionTests, VerticalVarianceChangesResults)
{
    Config cfg1{};
    cfg1.maze_size_scale = 1.0;
    cfg1.ir_reading_scale = 1.0;
    cfg1.mouse_angle = M_PI / 8;
    cfg1.horizontal_position_variance = 0.0;
    cfg1.vertical_position_variance = 1.0;
    cfg1.total_steps = 100;
    cfg1.reading_threshold = 900u;

    Config cfg2 {cfg1};
    cfg2.vertical_position_variance = -0.9;

    auto r1 {run_simulation(cfg1)};
    auto r2 {run_simulation(cfg2)};

    bool diff_found {false};

    for (size_t i {0}; i < r1.correct_detection_at_step.size(); ++i) {
        if (r1.correct_detection_at_step[i] != r2.correct_detection_at_step[i]) {
            diff_found = true;
            break;
        }
    }

    CHECK(diff_found);
}

IGNORE_TEST(WallDetectionTests, VisualizationDoesNotAffectResults)
{
    Config cfg{};
    cfg.maze_size_scale = 1.0;
    cfg.ir_reading_scale = 1.0;
    cfg.mouse_angle = 0.0;
    cfg.horizontal_position_variance = 0.0;
    cfg.vertical_position_variance = 0.0;
    cfg.total_steps = 50;
    cfg.reading_threshold = 750u;

    disable_visualization();
    auto r1 {run_simulation(cfg)};

    enable_visualization();
    auto r2 {run_simulation(cfg)};

    CHECK_EQUAL(r1.correct_detection_at_step.size(),
                r2.correct_detection_at_step.size());

    for (size_t i = 0; i < r1.correct_detection_at_step.size(); ++i) {
        CHECK_EQUAL(r1.correct_detection_at_step[i],
                    r2.correct_detection_at_step[i]);
    }
}
