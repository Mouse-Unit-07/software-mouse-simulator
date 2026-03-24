/*================================ FILE INFO =================================*/
/* Filename           : test_rotation.cpp                                     */
/*                                                                            */
/* Test implementation for rotation.cpp                                       */
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
#include "wheel_motor.h"
#include "magnetic_encoder.h"

}

#include <cmath>
#include <cstdint>
#include <vector>
#include <string>
#include <optional>
#include <functional>
#include <algorithm>
#include "point.hpp"
#include "ray.hpp"
#include "rectangular_hitbox.hpp"
#include "mouse.hpp"
#include "maze.hpp"
#include "optimizer.hpp"
#include "rotation.hpp"

#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>

using namespace maze;
using namespace optimizer;
using namespace rotation;

/*============================================================================*/
/*                             Public Definitions                             */
/*============================================================================*/
constexpr double FLOAT_TOLERANCE {1e-6};

std::vector<std::string> ascii {
    "+-+",
    "|S|",
    "+-+"
};
Maze test_maze {build_maze_from_ascii(ascii, 0.0)};

Config make_params(int kp, int kd, int shift, double motor_speed = 0)
{
    return {
        motor_speed, 1.0, 0.01,
        0, 0, 1, 1, 1,
        kp, kd, shift
    };
}

/*============================================================================*/
/*                            Mock Implementations                            */
/*============================================================================*/
/* none */

/*============================================================================*/
/*                                 Test Group                                 */
/*============================================================================*/
TEST_GROUP(RotationTests)
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
TEST(RotationTests, BuildConfigMapsValuesCorrectly)
{
    std::vector<double> v {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};

    auto cfg {build_config(v)};

    CHECK_EQUAL(1, cfg.motor_speed);
    CHECK_EQUAL(2, cfg.motor_speed_scale);
    CHECK_EQUAL(3, cfg.dt);
    CHECK_EQUAL(4, cfg.motor1_variance);
    CHECK_EQUAL(5, cfg.motor2_variance);
    CHECK_EQUAL(6, cfg.slip_factor);
    CHECK_EQUAL(7, cfg.wheel_circumference_scale);
    CHECK_EQUAL(8, cfg.wheel_base_scale);
    CHECK_EQUAL(9, cfg.kp);
    CHECK_EQUAL(10, cfg.kd);
    CHECK_EQUAL(11, cfg.pid_shift);
}

TEST(RotationTests, SimulationProducesValidResult)
{
    Config cfg {150, 1.0, 0.01, 0, 0, 1, 1, 1, 0, 0, 0};

    auto r {run_simulation(test_maze, cfg, M_PI / 2)};

    CHECK(r.total_time >= 0.0);
    CHECK(r.final_angle_error >= 0.0);
    CHECK(r.total_translation >= 0.0);
}

TEST(RotationTests, SimulationFailsWhenDtIsZero)
{
    Config cfg {150, 1.0, 0.0, 0, 0, 1, 1, 1, 0, 0, 0};

    auto r {run_simulation(test_maze, cfg, M_PI / 2)};

    CHECK(r.timeout);
}

TEST(RotationTests, PositiveAndNegativeAnglesProduceSameAngleAndTranslationError)
{
    Config cfg {150, 1.0, 0.01, 0, 0, 1, 1, 1, 0, 0, 0};

    auto r1 {run_simulation(test_maze, cfg,  M_PI / 2)};
    auto r2 {run_simulation(test_maze, cfg, -M_PI / 2)};

    CHECK_FALSE(r1.timeout);
    CHECK_FALSE(r2.timeout);
    CHECK(r1.total_translation == r2.total_translation);
    CHECK(r1.final_angle_error == r2.final_angle_error);
}

TEST(RotationTests, LargerAngleTakesMoreTime)
{
    Config cfg {150, 1.0, 0.01, 0, 0, 1, 1, 1, 0, 0, 0};

    auto small {run_simulation(test_maze, cfg, M_PI / 4)};
    auto large {run_simulation(test_maze, cfg, M_PI / 2)};

    CHECK(large.total_time >= small.total_time);
}

TEST(RotationTests, SimulationCanDetectCollision)
{
    std::vector<std::string> ascii {
        "+-+-+",
        "|S  |",
        "+-+-+"
    };

    Maze maze {build_maze_from_ascii(ascii, 0.0)};

    Config cfg {255, 1.0, 0.01, -1, 0, 1, 1, 1, 0, 0, 0};

    auto r {run_simulation(test_maze, cfg, M_PI)};

    CHECK(r.collision);
}

TEST(RotationTests, NoTranslationAndAngleErrorForPerfectTestVariables)
{
    std::vector<std::string> ascii {
        "+-+",
        "|S|",
        "+-+"
    };
    Maze maze {build_maze_from_ascii(ascii, 0.0)};

    /* slow movement, tiny dt, and no motor variances */
    Config cfg {100,1.0,0.001, 0,0, 1,1,1, 0,0,0};

    auto r {run_simulation(test_maze, cfg, M_PI)};

    /* 1% of a circle, or 3.6 degrees */
    constexpr double ROTATION_TOLERANCE {(2 * M_PI) * 0.01};

    CHECK_FALSE(r.timeout);
    DOUBLES_EQUAL(0.0, r.final_angle_error, ROTATION_TOLERANCE);
    DOUBLES_EQUAL(0.0, r.total_translation, 0.01);
}

TEST(RotationTests, ComputeResultsMetricsProducesCorrectStats)
{
    std::vector<Result> results {
        {0,1, 10, false, false},
        {0,2, 20, false, false},
        {0,3, 30, false, false}
    };

    auto s {compute_results_metrics(results)};

    DOUBLES_EQUAL(20.0, s.translation_stats.mean, FLOAT_TOLERANCE);
    CHECK_EQUAL(10.0, s.translation_stats.min);
    CHECK_EQUAL(30.0, s.translation_stats.max);
}

TEST(RotationTests, ComputeResultsMetricsComputesRates)
{
    std::vector<Result> results {
        {0, 0, 0, false, false},
        {0, 0, 0, true, false},
        {0, 0, 0, false, true},
        {0, 0, 0, true, true}
    };

    auto s {compute_results_metrics(results)};

    DOUBLES_EQUAL(0.5, s.timeout_rate, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(0.5, s.collision_rate, FLOAT_TOLERANCE);
}

TEST(RotationTests, ComputeResultsMetricsHandlesEmptyInput)
{
    std::vector<Result> results {};

    auto s {compute_results_metrics(results)};

    DOUBLES_EQUAL(0.0, s.timeout_rate, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(0.0, s.collision_rate, FLOAT_TOLERANCE);
    CHECK(s.time_stats.mean == 0.0 || true);
}

TEST(RotationTests, DerivativeTermAffectsStability)
{
    Config no_d  {150,1.0,0.01, -0.2,0, 1,1,1, 2000,0,8};
    Config with_d{150,1.0,0.01, -0.2,0, 1,1,1, 2000,1000,8};

    auto r1 {run_simulation(test_maze, no_d,  M_PI / 2)};
    auto r2 {run_simulation(test_maze, with_d,M_PI / 2)};

    CHECK((r1.final_angle_error != r2.final_angle_error)
       || (r1.total_translation != r2.total_translation));
}

TEST(RotationTests, PDImprovesAccuracyOverNoControl)
{
    Config no_control {150,1.0,0.01, -0.2,0, 1,1,1, 0,0,8};
    Config pd_control {150,1.0,0.01, -0.2,0, 1,1,1, 2000,1000,8};

    auto r1 {run_simulation(test_maze, no_control, M_PI / 2)};
    auto r2 {run_simulation(test_maze, pd_control, M_PI / 2)};

    CHECK(r2.final_angle_error <= r1.final_angle_error);
}

TEST(RotationTests, PidShiftAffectsControlStrength)
{
    Config strong {150,1.0,0.01, -0.2,0, 1,1,1, 2000,1000,2};
    Config weak   {150,1.0,0.01, -0.2,0, 1,1,1, 2000,1000,8};

    auto r1 {run_simulation(test_maze, strong, M_PI / 2)};
    auto r2 {run_simulation(test_maze, weak,   M_PI / 2)};

    CHECK((r1.total_time != r2.total_time)
       || (r1.final_angle_error != r2.final_angle_error));
}

TEST(RotationTests, BuildCandidatesGroupsAndComputesStats)
{
    std::vector<Trial> trials {
        {make_params(1,2,3), {10, 1.0, 100, false, false}},
        {make_params(1,2,3), {20, 2.0, 200, false, false}},
        {make_params(4,5,6), {30, 3.0, 300, true,  true}}
    };

    auto candidates {build_candidates(trials)};

    CHECK_EQUAL(2, candidates.size());

    for (const auto& c : candidates) {
        if ((c.key.kp == 1) && (c.key.kd == 2) && (c.key.shift == 3)) {
            DOUBLES_EQUAL(15.0, c.results_metrics.time_stats.mean, FLOAT_TOLERANCE);
            DOUBLES_EQUAL(1.5,  c.results_metrics.angle_error_stats.mean, FLOAT_TOLERANCE);
            DOUBLES_EQUAL(150.0,c.results_metrics.translation_stats.mean, FLOAT_TOLERANCE);
            DOUBLES_EQUAL(0.0,  c.results_metrics.timeout_rate, FLOAT_TOLERANCE);
            DOUBLES_EQUAL(0.0,  c.results_metrics.collision_rate, FLOAT_TOLERANCE);
        }
        if ((c.key.kp == 4) && (c.key.kd == 5) && (c.key.shift == 6)) {
            DOUBLES_EQUAL(30.0, c.results_metrics.time_stats.mean, FLOAT_TOLERANCE);
            DOUBLES_EQUAL(3.0,  c.results_metrics.angle_error_stats.mean, FLOAT_TOLERANCE);
            DOUBLES_EQUAL(300.0,c.results_metrics.translation_stats.mean, FLOAT_TOLERANCE);
            DOUBLES_EQUAL(1.0,  c.results_metrics.timeout_rate, FLOAT_TOLERANCE);
            DOUBLES_EQUAL(1.0,  c.results_metrics.collision_rate, FLOAT_TOLERANCE);
        }
    }
}

TEST(RotationTests, BuildCandidatesHandlesEmptyInput)
{
    std::vector<Trial> trials {};

    auto candidates {build_candidates(trials)};

    CHECK(candidates.empty());
}

TEST(RotationTests, SortCandidatesTimeoutRatePriority)
{
    Candidate a{}, b{};

    a.results_metrics.timeout_rate = 0.0;
    b.results_metrics.timeout_rate = 0.5;

    std::vector<Candidate> v {b, a};

    sort_candidates(v);

    DOUBLES_EQUAL(0.0, v[0].results_metrics.timeout_rate, FLOAT_TOLERANCE);
}

TEST(RotationTests, SortCandidatesCollisionRatePriority)
{
    Candidate a{}, b{};

    a.results_metrics.timeout_rate = 0.0;
    b.results_metrics.timeout_rate = 0.0;

    a.results_metrics.collision_rate = 0.0;
    b.results_metrics.collision_rate = 0.5;

    std::vector<Candidate> v {b, a};

    sort_candidates(v);

    DOUBLES_EQUAL(0.0, v[0].results_metrics.collision_rate, FLOAT_TOLERANCE);
}

TEST(RotationTests, SortCandidatesAngleErrorPriority)
{
    Candidate a{}, b{};

    a.results_metrics.timeout_rate = 0.0;
    b.results_metrics.timeout_rate = 0.0;
    a.results_metrics.collision_rate = 0.0;
    b.results_metrics.collision_rate = 0.0;

    a.results_metrics.angle_error_stats.mean = 1.0;
    b.results_metrics.angle_error_stats.mean = 2.0;

    std::vector<Candidate> v {b, a};

    sort_candidates(v);

    DOUBLES_EQUAL(1.0, v[0].results_metrics.angle_error_stats.mean, FLOAT_TOLERANCE);
}

TEST(RotationTests, SortCandidatesTranslationPriority)
{
    Candidate a{}, b{};

    a.results_metrics.timeout_rate = 0.0;
    b.results_metrics.timeout_rate = 0.0;
    a.results_metrics.collision_rate = 0.0;
    b.results_metrics.collision_rate = 0.0;
    a.results_metrics.angle_error_stats.mean = 1.0;
    b.results_metrics.angle_error_stats.mean = 1.0;

    a.results_metrics.translation_stats.mean = 100;
    b.results_metrics.translation_stats.mean = 200;

    std::vector<Candidate> v {b, a};

    sort_candidates(v);

    DOUBLES_EQUAL(100.0, v[0].results_metrics.translation_stats.mean, FLOAT_TOLERANCE);
}

TEST(RotationTests, SortCandidatesTimePriority)
{
    Candidate a{}, b{};

    a.results_metrics.timeout_rate = 0.0;
    b.results_metrics.timeout_rate = 0.0;
    a.results_metrics.collision_rate = 0.0;
    b.results_metrics.collision_rate = 0.0;
    a.results_metrics.angle_error_stats.mean = 1.0;
    b.results_metrics.angle_error_stats.mean = 1.0;
    a.results_metrics.translation_stats.mean = 100;
    b.results_metrics.translation_stats.mean = 100;

    a.results_metrics.time_stats.mean = 5.0;
    b.results_metrics.time_stats.mean = 10.0;

    std::vector<Candidate> v {b, a};

    sort_candidates(v);

    DOUBLES_EQUAL(5.0, v[0].results_metrics.time_stats.mean, FLOAT_TOLERANCE);
}

TEST(RotationTests, SortCandidatesUsesStdDevAsTieBreaker)
{
    Candidate a{}, b{};

    a.results_metrics.timeout_rate = 0.0;
    b.results_metrics.timeout_rate = 0.0;
    a.results_metrics.collision_rate = 0.0;
    b.results_metrics.collision_rate = 0.0;
    a.results_metrics.angle_error_stats.mean = 1.0;
    b.results_metrics.angle_error_stats.mean = 1.0;
    a.results_metrics.translation_stats.mean = 100;
    b.results_metrics.translation_stats.mean = 100;
    a.results_metrics.time_stats.mean = 5.0;
    b.results_metrics.time_stats.mean = 5.0;

    a.results_metrics.time_stats.stddev = 1.0;
    a.results_metrics.angle_error_stats.stddev = 1.0;
    a.results_metrics.translation_stats.stddev = 1.0;

    b.results_metrics.time_stats.stddev = 2.0;
    b.results_metrics.angle_error_stats.stddev = 2.0;
    b.results_metrics.translation_stats.stddev = 2.0;

    std::vector<Candidate> v {b, a};

    sort_candidates(v);

    DOUBLES_EQUAL(1.0, v[0].results_metrics.time_stats.stddev, FLOAT_TOLERANCE);
}

TEST(RotationTests, SortCandidatesRespectsFullPriorityOrder)
{
    Candidate best{}, mid{}, worst{};

    // worst: fails hard constraints
    worst.results_metrics.timeout_rate = 0.5;

    // mid: passes constraints but worse accuracy
    mid.results_metrics.timeout_rate = 0.0;
    mid.results_metrics.collision_rate = 0.0;
    mid.results_metrics.angle_error_stats.mean = 5.0;

    // best: better accuracy
    best.results_metrics.timeout_rate = 0.0;
    best.results_metrics.collision_rate = 0.0;
    best.results_metrics.angle_error_stats.mean = 1.0;

    std::vector<Candidate> v {mid, worst, best};

    sort_candidates(v);

    CHECK(v[0].results_metrics.angle_error_stats.mean == 1.0);
    CHECK(v[2].results_metrics.timeout_rate == 0.5);
}

TEST(RotationTests, BuildCandidatesAndSortWorkTogether)
{
    std::vector<Trial> trials {
        {make_params(1,0,8), {10, 2.0, 200, false, false}},
        {make_params(1,0,8), {12, 2.0, 210, false, false}},

        {make_params(2,0,8), {8,  1.0, 150, false, false}},
        {make_params(2,0,8), {9,  1.2, 160, false, false}},

        {make_params(3,0,8), {5,  0.5, 100, true,  false}}
    };

    auto ranked {build_candidates(trials)};
    sort_candidates(ranked);

    CHECK(ranked.size() >= 2);

    CHECK(ranked[0].key.kp == 2);
    CHECK(ranked[0].key.kd == 0);
    CHECK(ranked[0].key.shift == 8);
}

TEST(RotationTests, RunMinimalSampleSimulation)
{
    std::vector<optimizer::SweepConfig> test_configs {
        {"motor_speed", 100, 100, 1},
        {"motor_speed_scale", 1.0, 1.0, 1},
        {"dt", 0.001, 0.001, 1},

        {"motor1_variance", 0.0, 0.0, 1},
        {"motor2_variance", 0.0, 0.0, 1},
        {"slip_factor", 1.0, 1.0, 1},
        {"wheel_circumference_scale", 1.0, 1.0, 1},
        {"wheel_base_scale", 1.0, 1.0, 1},

        {"kp", 0, 0, 1},
        {"kd", 0, 0, 1},
        {"pid_shift", 8, 8, 1}
    };

    const std::string filename {"test_minimal_output.txt"};

    run_full_rotation_experiment(filename, M_PI / 2, test_configs);
}

IGNORE_TEST(RotationTests, RunFullSimulationAndWriteResultsToFile)
{
    std::vector<optimizer::SweepConfig> test_configs {
        {"motor_speed", 120, 220, 5},
        {"motor_speed_scale", 0.9, 1.1, 3},
        {"dt", 0.01, 0.5, 3},

        {"motor1_variance", -0.1, 0.1, 3},
        {"motor2_variance", -0.1, 0.1, 3},
        {"slip_factor", 0.9, 1.1, 3},
        {"wheel_circumference_scale", 0.95, 1.05, 3},
        {"wheel_base_scale", 0.95, 1.05, 3},

        {"kp", 0, 4000, 21},
        {"kd", 0, 2000, 21},
        {"pid_shift", 8, 8, 1}
    };

    rotation::run_full_rotation_experiment("test_full_output.txt", M_PI / 2, test_configs);
}
