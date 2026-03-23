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

std::vector<double> make_params(int kp, int kd, int shift, double motor_speed = 0)
{
    return {
        motor_speed, 1.0, 0.01,
        0, 0, 1, 1, 1,
        static_cast<double>(kp),
        static_cast<double>(kd),
        static_cast<double>(shift)
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
TEST(RotationTests, BuildRotationConfigMapsValuesCorrectly)
{
    std::vector<double> v {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};

    auto cfg {build_rotation_config(v)};

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
    RotationConfig cfg {150, 1.0, 0.01, 0, 0, 1, 1, 1, 0, 0, 0};

    auto r {run_rotation_simulation(test_maze, cfg, M_PI / 2)};

    CHECK(r.total_time >= 0.0);
    CHECK(r.final_angle_error >= 0.0);
    CHECK(r.total_translation >= 0.0);
}

TEST(RotationTests, SimulationFailsWhenDtIsZero)
{
    RotationConfig cfg {150, 1.0, 0.0, 0, 0, 1, 1, 1, 0, 0, 0};

    auto r {run_rotation_simulation(test_maze, cfg, M_PI / 2)};

    CHECK(r.simulation_failed);
}

TEST(RotationTests, PositiveAndNegativeAnglesProduceSameAngleAndTranslationError)
{
    RotationConfig cfg {150, 1.0, 0.01, 0, 0, 1, 1, 1, 0, 0, 0};

    auto r1 {run_rotation_simulation(test_maze, cfg,  M_PI / 2)};
    auto r2 {run_rotation_simulation(test_maze, cfg, -M_PI / 2)};

    CHECK_FALSE(r1.simulation_failed);
    CHECK_FALSE(r2.simulation_failed);
    CHECK(r1.total_translation == r2.total_translation);
    CHECK(r1.final_angle_error == r2.final_angle_error);
}

TEST(RotationTests, LargerAngleTakesMoreTime)
{
    RotationConfig cfg {150, 1.0, 0.01, 0, 0, 1, 1, 1, 0, 0, 0};

    auto small {run_rotation_simulation(test_maze, cfg, M_PI / 4)};
    auto large {run_rotation_simulation(test_maze, cfg, M_PI / 2)};

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

    RotationConfig cfg {255, 1.0, 0.01, -1, 0, 1, 1, 1, 0, 0, 0};

    auto r {run_rotation_simulation(test_maze, cfg, M_PI)};

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
    RotationConfig cfg {100,1.0,0.001, 0,0, 1,1,1, 0,0,0};

    auto r {run_rotation_simulation(test_maze, cfg, M_PI)};

    /* 1% of a circle, or 3.6 degrees */
    constexpr double ROTATION_TOLERANCE {(2 * M_PI) * 0.01};

    CHECK_FALSE(r.simulation_failed);
    DOUBLES_EQUAL(0.0, r.final_angle_error, ROTATION_TOLERANCE);
    DOUBLES_EQUAL(0.0, r.total_translation, 0.01);
}

TEST(RotationTests, AnalyzeResultsComputesStats)
{
    std::vector<std::pair<std::vector<double>, RotationResult>> trials {
        {{}, {0,1, 10, false, false}},
        {{}, {0,2, 20, false, false}},
        {{}, {0,3, 30, false, false}}
    };

    auto s {analyze_rotation_results(trials)};

    DOUBLES_EQUAL(20.0, s.translation_stats.mean, FLOAT_TOLERANCE);
    CHECK_EQUAL(10.0, s.translation_stats.min);
    CHECK_EQUAL(30.0, s.translation_stats.max);
}

TEST(RotationTests, AnalyzeResultsComputesRates)
{
    std::vector<std::pair<std::vector<double>, RotationResult>> trials {
        {{}, {0, 0, 0, false, false}},
        {{}, {0, 0, 0, true, false}},
        {{}, {0, 0, 0, false, true}},
        {{}, {0, 0, 0, true, true}}
    };

    auto s {analyze_rotation_results(trials)};

    DOUBLES_EQUAL(0.5, s.failure_rate, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(0.5, s.collision_rate, FLOAT_TOLERANCE);
}

TEST(RotationTests, DerivativeTermAffectsStability)
{
    RotationConfig no_d  {150,1.0,0.01, -0.2,0, 1,1,1, 2000,0,8};
    RotationConfig with_d{150,1.0,0.01, -0.2,0, 1,1,1, 2000,1000,8};

    auto r1 {run_rotation_simulation(test_maze, no_d,  M_PI / 2)};
    auto r2 {run_rotation_simulation(test_maze, with_d,M_PI / 2)};

    CHECK((r1.final_angle_error != r2.final_angle_error)
       || (r1.total_translation != r2.total_translation));
}

TEST(RotationTests, PDImprovesAccuracyOverNoControl)
{
    RotationConfig no_control {150,1.0,0.01, -0.2,0, 1,1,1, 0,0,8};
    RotationConfig pd_control {150,1.0,0.01, -0.2,0, 1,1,1, 2000,1000,8};

    auto r1 {run_rotation_simulation(test_maze, no_control, M_PI / 2)};
    auto r2 {run_rotation_simulation(test_maze, pd_control, M_PI / 2)};

    CHECK(r2.final_angle_error <= r1.final_angle_error);
}

TEST(RotationTests, PidShiftAffectsControlStrength)
{
    RotationConfig strong {150,1.0,0.01, -0.2,0, 1,1,1, 2000,1000,2};
    RotationConfig weak   {150,1.0,0.01, -0.2,0, 1,1,1, 2000,1000,8};

    auto r1 {run_rotation_simulation(test_maze, strong, M_PI / 2)};
    auto r2 {run_rotation_simulation(test_maze, weak,   M_PI / 2)};

    CHECK((r1.total_time != r2.total_time)
       || (r1.final_angle_error != r2.final_angle_error));
}

TEST(RotationTests, AnalyzePdCandidatesGroupsAndComputesStats)
{
    std::vector<std::pair<std::vector<double>, RotationResult>> trials {
        {make_params(1,2,3), {10, 1.0, 100, false, false}},
        {make_params(1,2,3), {20, 2.0, 200, false, false}},
        {make_params(4,5,6), {30, 3.0, 300, true,  true}}
    };

    auto candidates {analyze_pd_candidates(trials)};

    CHECK_EQUAL(2, candidates.size());

    for (const auto& c : candidates) {
        if ((c.key.kp == 1) && (c.key.kd == 2) && (c.key.shift == 3)) {
            DOUBLES_EQUAL(15.0, c.time_stats.mean, FLOAT_TOLERANCE);
            DOUBLES_EQUAL(1.5,  c.angle_error_stats.mean, FLOAT_TOLERANCE);
            DOUBLES_EQUAL(150.0,c.translation_stats.mean, FLOAT_TOLERANCE);
            DOUBLES_EQUAL(0.0,  c.failure_rate, FLOAT_TOLERANCE);
            DOUBLES_EQUAL(0.0,  c.collision_rate, FLOAT_TOLERANCE);
        }
    }
}

TEST(RotationTests, SortCandidatesFailureRatePriority)
{
    RotationCandidate a{}, b{};

    a.failure_rate = 0.0;
    b.failure_rate = 0.5;

    std::vector<RotationCandidate> v {b, a};

    sort_rotation_candidates(v);

    DOUBLES_EQUAL(0.0, v[0].failure_rate, FLOAT_TOLERANCE);
}

TEST(RotationTests, SortCandidatesCollisionRatePriority)
{
    RotationCandidate a{}, b{};

    a.failure_rate = b.failure_rate = 0.0;

    a.collision_rate = 0.0;
    b.collision_rate = 0.5;

    std::vector<RotationCandidate> v {b, a};

    sort_rotation_candidates(v);

    DOUBLES_EQUAL(0.0, v[0].collision_rate, FLOAT_TOLERANCE);
}

TEST(RotationTests, SortCandidatesAngleErrorPriority)
{
    RotationCandidate a{}, b{};

    a.failure_rate = 0.0;
    b.failure_rate = 0.0;
    a.collision_rate = 0.0;
    b.collision_rate = 0.0;

    a.angle_error_stats.mean = 1.0;
    b.angle_error_stats.mean = 2.0;

    std::vector<RotationCandidate> v {b, a};

    sort_rotation_candidates(v);

    DOUBLES_EQUAL(1.0, v[0].angle_error_stats.mean, FLOAT_TOLERANCE);
}

TEST(RotationTests, SortCandidatesTranslationPriority)
{
    RotationCandidate a{}, b{};

    a.failure_rate = 0.0;
    b.failure_rate = 0.0;
    a.collision_rate = 0.0;
    b.collision_rate = 0.0;
    a.angle_error_stats.mean = 1.0;
    b.angle_error_stats.mean = 1.0;

    a.translation_stats.mean = 100;
    b.translation_stats.mean = 200;

    std::vector<RotationCandidate> v {b, a};

    sort_rotation_candidates(v);

    DOUBLES_EQUAL(100.0, v[0].translation_stats.mean, FLOAT_TOLERANCE);
}

TEST(RotationTests, SortCandidatesTimePriority)
{
    RotationCandidate a{}, b{};

    a.failure_rate = 0.0;
    b.failure_rate = 0.0;
    a.collision_rate = 0.0;
    b.collision_rate = 0.0;
    a.angle_error_stats.mean = 1.0;
    b.angle_error_stats.mean = 1.0;
    a.translation_stats.mean = 100;
    b.translation_stats.mean = 100;

    a.time_stats.mean = 5.0;
    b.time_stats.mean = 10.0;

    std::vector<RotationCandidate> v {b, a};

    sort_rotation_candidates(v);

    DOUBLES_EQUAL(5.0, v[0].time_stats.mean, FLOAT_TOLERANCE);
}

TEST(RotationTests, SortCandidatesUsesStdDevAsTieBreaker)
{
    RotationCandidate a{}, b{};

    a.failure_rate = 0.0;
    b.failure_rate = 0.0;
    a.collision_rate = 0.0;
    b.collision_rate = 0.0;
    a.angle_error_stats.mean = 1.0;
    b.angle_error_stats.mean = 1.0;
    a.translation_stats.mean = 100;
    b.translation_stats.mean = 100;
    a.time_stats.mean = 5.0;
    b.time_stats.mean = 5.0;

    a.time_stats.stddev = 1.0;
    a.angle_error_stats.stddev = 1.0;
    a.translation_stats.stddev = 1.0;

    b.time_stats.stddev = 2.0;
    b.angle_error_stats.stddev = 2.0;
    b.translation_stats.stddev = 2.0;

    std::vector<RotationCandidate> v {b, a};

    sort_rotation_candidates(v);

    DOUBLES_EQUAL(1.0, v[0].time_stats.stddev, FLOAT_TOLERANCE);
}

TEST(RotationTests, GetRankedPdCandidatesOrdersCorrectly)
{
    std::vector<std::pair<std::vector<double>, RotationResult>> trials {
        {make_params(1,0,8), {10, 2.0, 200, false, false}},
        {make_params(1,0,8), {12, 2.0, 210, false, false}},

        {make_params(2,0,8), {8,  1.0, 150, false, false}},
        {make_params(2,0,8), {9,  1.2, 160, false, false}},

        {make_params(3,0,8), {5,  0.5, 100, true,  false}}
    };

    auto ranked {get_ranked_pd_candidates(trials)};

    CHECK(ranked.size() >= 2);

    CHECK(ranked[0].key.kp == 2);
    CHECK(ranked[0].key.kd == 0);
    CHECK(ranked[0].key.shift == 8);
}

IGNORE_TEST(RotationTests, RunDefaultSimulationAndPrintResults)
{
    rotation::run_full_rotation_experiment(M_PI / 2);
}
