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
Config create_no_variance_config(void)
{
    Config cfg{};
    cfg.maze_size_scale = 1.0;
    cfg.ir_reading_scale = 1.0;
    cfg.mouse_angle = 0.0;
    cfg.horizontal_position_variance = 0.0;
    cfg.vertical_position_variance = 0.0;
    cfg.total_steps = 100;
    cfg.reading_threshold = 750u; /* arbitrary threshold */

    return cfg;
}

ConfigSweeper create_no_variance_sweeper(void)
{
    ConfigSweeper sweeper;

    sweeper.maze_size_scale = {1.0};
    sweeper.ir_reading_scale = {1.0};
    sweeper.mouse_angle = {0.0};
    sweeper.horizontal_position_variance = {0.0};
    sweeper.vertical_position_variance = {0.0};
    sweeper.total_steps = {100};

    sweeper.reading_threshold = {750u};

    return sweeper;
}

bool are_results_equivalent(const Result& r1, const Result& r2)
{
    if (r1.wall_absent_at_step.size() != r2.wall_absent_at_step.size()) {
        return false;
    }
    if (r1.wall_present_at_step.size() != r2.wall_present_at_step.size()) {
        return false;
    }

    for (size_t i {0}; i < r1.wall_absent_at_step.size(); ++i) {
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
TEST(WallDetectionTests, ConfigSweeperProducesFirstValue)
{
    ConfigSweeper sweeper{create_no_variance_sweeper()};

    CHECK(sweeper.next());

    auto cfg {sweeper.value()};

    CHECK_EQUAL(1.0, cfg.maze_size_scale);
    CHECK_EQUAL(750u, cfg.reading_threshold);
}

TEST(WallDetectionTests, ConfigSweeperIteratesAllCombinations)
{
    ConfigSweeper sweeper{create_no_variance_sweeper()};
    sweeper.maze_size_scale = {1.0, 1.05};
    sweeper.reading_threshold = {100u, 200u};

    int count {0};

    while (sweeper.next()) {
        sweeper.value();
        count++;
    }

    CHECK_EQUAL(4, count); /* 2 maze_size_scale * 2 reading_threshold */
}

TEST(WallDetectionTests, ConfigSweeperStopsAtEnd)
{
    ConfigSweeper sweeper{create_no_variance_sweeper()};

    CHECK(sweeper.next());
    CHECK_FALSE(sweeper.next());
}

TEST(WallDetectionTests, ConfigSweeperOrderIsStable)
{
    ConfigSweeper sweeper{create_no_variance_sweeper()};
    sweeper.maze_size_scale = {1.0, 1.05};
    sweeper.reading_threshold = {100u, 200u};

    std::vector<std::pair<double, uint32_t>> seen;

    while (sweeper.next()) {
        auto cfg {sweeper.value()};
        seen.emplace_back(cfg.maze_size_scale, cfg.reading_threshold);
    }

    CHECK_EQUAL(4, seen.size());

    /* Expected order: maze_size_scale outer, reading_threshold inner */
    CHECK(seen[0] == std::make_pair(1.0, 100u));
    CHECK(seen[1] == std::make_pair(1.0, 200u));
    CHECK(seen[2] == std::make_pair(1.05, 100u));
    CHECK(seen[3] == std::make_pair(1.05, 200u));
}

TEST(WallDetectionTests, SimulationHandlesZeroSteps)
{
    Config cfg{create_no_variance_config()};
    cfg.total_steps = 0;

    auto result {run_simulation(cfg)};

    CHECK_EQUAL(0, result.wall_absent_at_step.size());
    CHECK_EQUAL(0, result.wall_present_at_step.size());
}

TEST(WallDetectionTests, WallAbsentAllFalseWhenThresholdIsZero)
{
    Config cfg{create_no_variance_config()};
    cfg.reading_threshold = 0u;  /* nothing should pass */

    auto result {run_simulation(cfg)};

    for (bool v : result.wall_absent_at_step) {
        CHECK_FALSE(v);
    }
}

TEST(WallDetectionTests, WallPresentAllTrueWhenThresholdIsZero)
{
    Config cfg{create_no_variance_config()};
    cfg.reading_threshold = 0u;  /* everything should pass */

    auto result {run_simulation(cfg)};

    for (bool v : result.wall_present_at_step) {
        CHECK(v);
    }
}

TEST(WallDetectionTests, IdenticalConfigsProduceIdenticalResults)
{
    Config cfg{create_no_variance_config()};

    auto r1 {run_simulation(cfg)};
    auto r2 {run_simulation(cfg)};

    CHECK(are_results_equivalent(r1, r2));
}

TEST(WallDetectionTests, MazeSizeScaleChangesResults)
{
    Config cfg1{create_no_variance_config()};
    Config cfg2 {cfg1};
    cfg2.maze_size_scale = 10.0;

    auto r1 {run_simulation(cfg1)};
    auto r2 {run_simulation(cfg2)};

    CHECK(!are_results_equivalent(r1, r2));
}

TEST(WallDetectionTests, IrReadingScaleChangesResults)
{
    Config cfg1{create_no_variance_config()};
    Config cfg2 {cfg1};
    cfg2.ir_reading_scale = 0.5;

    auto r1 {run_simulation(cfg1)};
    auto r2 {run_simulation(cfg2)};

    CHECK(!are_results_equivalent(r1, r2));
}

TEST(WallDetectionTests, ZeroIrReadingScaleCollapsesToAllTrueWhenThresholdIsMax)
{
    Config cfg{create_no_variance_config()};
    cfg.reading_threshold = 1024u;

    auto result {run_simulation(cfg)};

    for (bool v : result.wall_absent_at_step) {
        CHECK(v);
    }
}

TEST(WallDetectionTests, MouseAngleChangesResults)
{
    Config cfg1{create_no_variance_config()};
    Config cfg2 {cfg1};
    cfg2.mouse_angle = M_PI / 2;

    auto r1 {run_simulation(cfg1)};
    auto r2 {run_simulation(cfg2)};

    CHECK(!are_results_equivalent(r1, r2));
}

TEST(WallDetectionTests, HorizontalVarianceChangesResults)
{
    Config cfg1{create_no_variance_config()};
    Config cfg2 {cfg1};
    cfg2.horizontal_position_variance = -0.9;

    auto r1 {run_simulation(cfg1)};
    auto r2 {run_simulation(cfg2)};

    CHECK(!are_results_equivalent(r1, r2));
}

TEST(WallDetectionTests, VerticalVarianceChangesResults)
{
    Config cfg1{create_no_variance_config()};
    Config cfg2 {cfg1};
    cfg2.vertical_position_variance = -0.9;

    auto r1 {run_simulation(cfg1)};
    auto r2 {run_simulation(cfg2)};

    CHECK(!are_results_equivalent(r1, r2));
}

TEST(WallDetectionTests, VisualizationDoesNotAffectResults)
{
    Config cfg{create_no_variance_config()};

    disable_visualization();
    auto r1 {run_simulation(cfg)};

    enable_visualization();
    auto r2 {run_simulation(cfg)};

    CHECK(are_results_equivalent(r1, r2));
}

TEST(WallDetectionTests, ComputeResultsMetricsEmpty)
{
    std::vector<Result> results;

    auto m {compute_results_metrics(results)};

    CHECK_EQUAL(-1, m.detection_window.window_start);
    CHECK_EQUAL(0,  m.detection_window.window_size);
}

TEST(WallDetectionTests, ComputeResultsMetricsCountsConsensusCorrectly)
{
    Result r1;
    r1.wall_absent_at_step  = {true, true, false, true, true, true};
    r1.wall_present_at_step = {true, true, true, true, true, true};

    Result r2 {r1};

    std::vector<Result> results{r1, r2};

    auto m {compute_results_metrics(results)};

    std::vector<int> expected {2, 2, 0, 2, 2, 2};

    for (size_t i = 0; i < expected.size(); ++i) {
        CHECK_EQUAL(expected[i], m.correct_detection_count_at_step[i]);
    }
}

TEST(WallDetectionTests, BuildCandidatesGroupsByThreshold)
{
    Trial t1;
    t1.result.wall_absent_at_step  = {true, true};
    t1.result.wall_present_at_step = {true, true};
    t1.config.reading_threshold = 100;

    Trial t2 {t1};

    Trial t3;
    t3.result.wall_absent_at_step  = {false, false};
    t3.result.wall_present_at_step = {true, true};
    t3.config.reading_threshold = 200;

    std::vector<Trial> trials{t1, t2, t3};

    auto candidates {build_candidates(trials)};

    CHECK_EQUAL(2, candidates.size());
}

TEST(WallDetectionTests, BuildCandidatesDoesNotComputeMetricsPerGroup)
{
    Trial t1;
    t1.result.wall_absent_at_step  = {true, true, false};
    t1.result.wall_present_at_step = {true, true, true};
    t1.config.reading_threshold = 100;

    Trial t2 {t1};

    std::vector<Trial> trials{t1, t2};

    auto candidates {build_candidates(trials)};

    CHECK_EQUAL(1, candidates.size());

    const auto& m {candidates[0].results_metrics};

    CHECK_EQUAL(-1, m.detection_window.window_start);
    CHECK_EQUAL(0, m.detection_window.window_size);
}

TEST(WallDetectionTests, FilterCandidatesPerfectDetection)
{
    Candidate c;
    c.key.threshold = 100;
    c.results_metrics.correct_detection_count_at_step = {2, 2, 0, 2};
    c.results_metrics.total_detection_counts_per_step = 2;

    std::vector<Candidate> candidates {c};

    auto filtered {filter_candidates_by_rate(candidates, 1.0)};

    CHECK_EQUAL(1, filtered.size());
    CHECK_EQUAL(0, filtered[0].results_metrics.detection_window.window_start);
    CHECK_EQUAL(2, filtered[0].results_metrics.detection_window.window_size);
}

TEST(WallDetectionTests, FilterCandidatesPartialCorrectDetection)
{
    Candidate c;
    c.key.threshold = 100;
    c.results_metrics.correct_detection_count_at_step = {2, 1, 1, 2};
    c.results_metrics.total_detection_counts_per_step = 2;

    std::vector<Candidate> candidates {c};

    auto filtered {filter_candidates_by_rate(candidates, 0.5)};

    CHECK_EQUAL(1, filtered.size());
    CHECK_EQUAL(0, filtered[0].results_metrics.detection_window.window_start);
    CHECK_EQUAL(4, filtered[0].results_metrics.detection_window.window_size);
}

TEST(WallDetectionTests, FilterCandidatesNoValidWindow)
{
    Candidate c;
    c.key.threshold = 100;
    c.results_metrics.correct_detection_count_at_step = {0, 0, 1};
    c.results_metrics.total_detection_counts_per_step = 2;

    std::vector<Candidate> candidates {c};

    auto filtered {filter_candidates_by_rate(candidates, 0.75)};

    CHECK_EQUAL(0, filtered.size());
}

IGNORE_TEST(WallDetectionTests, RunFullSimulationAndWriteResultsToFile)
{
    ConfigSweeper sweeper;

    sweeper.maze_size_scale = simulation_common::generate_sweep_values(0.95, 1.05, 3);
    sweeper.ir_reading_scale = simulation_common::generate_sweep_values(0.95, 1.05, 3);
    sweeper.mouse_angle = simulation_common::generate_sweep_values(-M_PI / 8, M_PI / 8, 3);
    sweeper.horizontal_position_variance = simulation_common::generate_sweep_values(-0.5, 0.5, 3);
    sweeper.vertical_position_variance = simulation_common::generate_sweep_values(-0.5, 0.5, 3);
    sweeper.total_steps = {100};
    sweeper.reading_threshold = simulation_common::generate_sweep_values<uint32_t>(700, 1024, 225);

    run_full_wall_detection_experiment("test_full_output.txt", sweeper, 0.5);
}
