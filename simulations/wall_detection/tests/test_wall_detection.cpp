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
#include <functional>
#include "point.hpp"
#include "ray.hpp"
#include "rectangular_hitbox.hpp"
#include "mouse.hpp"
#include "maze.hpp"
#include "simulation_common.hpp"
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

    CHECK_EQUAL(0, result.wall_absent_at_step.size());
    CHECK_EQUAL(0, result.wall_present_at_step.size());
}

TEST(WallDetectionTests, WallAbsentAllFalseWhenThresholdIsZero)
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

    for (bool v : result.wall_absent_at_step) {
        CHECK_FALSE(v);
    }
}

TEST(WallDetectionTests, WallPresentAllTrueWhenThresholdIsZero)
{
    Config cfg{};
    cfg.maze_size_scale = 1.0;
    cfg.ir_reading_scale = 1.0;
    cfg.mouse_angle = 0.0;
    cfg.horizontal_position_variance = 0.0;
    cfg.vertical_position_variance = 0.0;
    cfg.total_steps = 100;
    cfg.reading_threshold = 0u;  /* everything should pass */

    auto result {run_simulation(cfg)};

    for (bool v : result.wall_present_at_step) {
        CHECK(v);
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

    CHECK_EQUAL(r1.wall_absent_at_step.size(), r2.wall_absent_at_step.size());
    CHECK_EQUAL(r1.wall_present_at_step.size(), r2.wall_present_at_step.size());

    for (size_t i {0}; i < r1.wall_absent_at_step.size(); ++i) {
        CHECK_EQUAL(r1.wall_absent_at_step[i], r2.wall_absent_at_step[i]);
        CHECK_EQUAL(r1.wall_present_at_step[i], r2.wall_present_at_step[i]);
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

    for (size_t i {0}; i < r1.wall_absent_at_step.size(); ++i) {
        if (r1.wall_absent_at_step[i] != r2.wall_absent_at_step[i]) {
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

    for (size_t i {0}; i < r1.wall_absent_at_step.size(); ++i) {
        if (r1.wall_absent_at_step[i] != r2.wall_absent_at_step[i]) {
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

    for (bool v : result.wall_absent_at_step) {
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

    for (size_t i {0}; i < r1.wall_absent_at_step.size(); ++i) {
        if (r1.wall_absent_at_step[i] != r2.wall_absent_at_step[i]) {
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

    for (size_t i {0}; i < r1.wall_absent_at_step.size(); ++i) {
        if (r1.wall_absent_at_step[i] != r2.wall_absent_at_step[i]) {
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

    for (size_t i {0}; i < r1.wall_absent_at_step.size(); ++i) {
        if (r1.wall_absent_at_step[i] != r2.wall_absent_at_step[i]) {
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

    CHECK_EQUAL(r1.wall_absent_at_step.size(), r2.wall_absent_at_step.size());

    for (size_t i = 0; i < r1.wall_absent_at_step.size(); ++i) {
        CHECK_EQUAL(r1.wall_absent_at_step[i], r2.wall_absent_at_step[i]);
        CHECK_EQUAL(r1.wall_present_at_step[i], r2.wall_present_at_step[i]);
    }
}

TEST(WallDetectionTests, ComputeResultsMetricsEmpty)
{
    std::vector<Result> results;

    auto m {compute_results_metrics(results)};

    CHECK_EQUAL(-1, m.window_start);
    CHECK_EQUAL(0,  m.window_size);
}

TEST(WallDetectionTests, ComputeResultsMetricsNoConsensus)
{
    Result r1;
    r1.wall_absent_at_step  = {true, false, true};
    r1.wall_present_at_step = {true, true, true};

    Result r2;
    r2.wall_absent_at_step  = {false, true, false};
    r2.wall_present_at_step = {true, true, true};

    std::vector<Result> results{r1, r2};

    auto m {compute_results_metrics(results)};

    CHECK_EQUAL(-1, m.window_start);
    CHECK_EQUAL(0,  m.window_size);
}

TEST(WallDetectionTests, ComputeResultsMetricsSingleWindow)
{
    Result r1;
    r1.wall_absent_at_step  = {false, true, true, false};
    r1.wall_present_at_step = {true, true, true, true};

    Result r2 = r1;

    std::vector<Result> results{r1, r2};

    auto m {compute_results_metrics(results)};

    CHECK_EQUAL(1, m.window_start);
    CHECK_EQUAL(2, m.window_size);
}

TEST(WallDetectionTests, ComputeResultsMetricsPicksLongestWindow)
{
    Result r1;
    r1.wall_absent_at_step  = {true, true, false, true, true, true};
    r1.wall_present_at_step = {true, true, true, true, true, true};

    Result r2 = r1;

    std::vector<Result> results{r1, r2};

    auto m {compute_results_metrics(results)};

    CHECK_EQUAL(3, m.window_start);
    CHECK_EQUAL(3, m.window_size);
}

TEST(WallDetectionTests, ComputeResultsMetricsRequiresAllTrue)
{
    Result r1;
    r1.wall_absent_at_step  = {true, true, true};
    r1.wall_present_at_step = {true, true, true};

    Result r2;
    r2.wall_absent_at_step  = {true, false, true};
    r2.wall_present_at_step = {true, true, true};

    std::vector<Result> results{r1, r2};

    auto m {compute_results_metrics(results)};

    CHECK_EQUAL(0, m.window_start);
    CHECK_EQUAL(1, m.window_size);
}

TEST(WallDetectionTests, ComputeResultsMetricsFailsIfWallPresentIsFalse)
{
    Result r1;
    r1.wall_absent_at_step  = {true, true, true};
    r1.wall_present_at_step = {false, false, false};

    std::vector<Result> results{r1};

    auto m {compute_results_metrics(results)};

    CHECK_EQUAL(-1, m.window_start);
    CHECK_EQUAL(0,  m.window_size);
}

TEST(WallDetectionTests, ComputeResultsMetricsRequiresBothSignalsTrue)
{
    Result r1;
    r1.wall_absent_at_step  = {true, true, false};
    r1.wall_present_at_step = {true, true, true};

    std::vector<Result> results{r1};

    auto m {compute_results_metrics(results)};

    CHECK_EQUAL(0, m.window_start);
    CHECK_EQUAL(2, m.window_size);
}

TEST(WallDetectionTests, BuildCandidatesGroupsByThreshold)
{
    Trial t1;
    t1.result.wall_absent_at_step  = {true, true};
    t1.result.wall_present_at_step = {true, true};
    t1.config.reading_threshold = 100;

    Trial t2 = t1;

    Trial t3;
    t3.result.wall_absent_at_step  = {false, false};
    t3.result.wall_present_at_step = {true, true};
    t3.config.reading_threshold = 200;

    std::vector<Trial> trials{t1, t2, t3};

    auto candidates {build_candidates(trials)};

    CHECK_EQUAL(2, candidates.size());
}

TEST(WallDetectionTests, BuildCandidatesComputesMetricsPerGroup)
{
    Trial t1;
    t1.result.wall_absent_at_step  = {true, true, false};
    t1.result.wall_present_at_step = {true, true, true};
    t1.config.reading_threshold = 100;

    Trial t2 = t1;

    std::vector<Trial> trials{t1, t2};

    auto candidates {build_candidates(trials)};

    CHECK_EQUAL(1, candidates.size());

    const auto& m = candidates[0].results_metrics;

    CHECK_EQUAL(0, m.window_start);
    CHECK_EQUAL(2, m.window_size);
}

TEST(WallDetectionTests, SortCandidatesByThresholdAscending)
{
    Candidate c1{{300}, {}};
    Candidate c2{{100}, {}};
    Candidate c3{{200}, {}};

    std::vector<Candidate> v{c1, c2, c3};

    sort_candidates_by_lowest_threshold(v);

    CHECK_EQUAL(100, v[0].key.threshold);
    CHECK_EQUAL(200, v[1].key.threshold);
    CHECK_EQUAL(300, v[2].key.threshold);
}

IGNORE_TEST(WallDetectionTests, RunFullSimulationAndWriteResultsToFile)
{
    std::vector<simulation_common::SweepConfig> test_configs {
        {"maze_size_scale", 1.0, 1.0, 1}, // 0.95, 1.05, 3
        {"ir_reading_scale", 1.0, 1.0, 1}, // 0.95, 1.05, 3
        {"mouse_angle", -M_PI / 64, M_PI / 64, 3},
        {"horizontal_position_variance", -0.9, 0.9, 3},
        {"vertical_position_variance", -0.9, 0.9, 3},
        {"total_steps", 100, 100, 1},

        {"reading_threshold", 800, 1024, 225},
    };

    run_full_wall_detection_experiment("test_full_output.txt", test_configs);
}
