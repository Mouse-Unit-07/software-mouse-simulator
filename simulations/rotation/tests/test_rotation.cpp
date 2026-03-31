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
#include <map>
#include <fstream>
#include <sstream>
#include "point.hpp"
#include "ray.hpp"
#include "rectangular_hitbox.hpp"
#include "mouse.hpp"
#include "maze.hpp"
#include "simulation_common.hpp"
#include "rotation.hpp"

#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>

using namespace rotation;

/*============================================================================*/
/*                             Public Definitions                             */
/*============================================================================*/
constexpr double FLOAT_TOLERANCE {1e-6};

std::vector<std::string> ascii{
    "+-+",
    "|S|",
    "+-+"
};
maze::Maze test_maze{maze::build_maze_from_ascii(ascii, 0.0)};

Config create_no_variance_config(void)
{
    Config cfg;

    cfg.motor_speed = 150u;
    cfg.motor_speed_scale = 1.0;
    cfg.dt = 0.01;
    cfg.motor1_variance = 0.0;
    cfg.motor2_variance = 0.0;
    cfg.slip_factor = 1.0;
    cfg.wheel_circumference_scale = 1.0;
    cfg.wheel_base_scale = 1.0;
    cfg.kp = 0;
    cfg.kd = 0;
    cfg.pid_shift = 0;

    return cfg;
}

Config create_config_custom_pid_and_speed(int kp, int kd, int shift, uint8_t motor_speed = 0)
{
    Config cfg{create_no_variance_config()};
    
    cfg.kp = kp;
    cfg.kd = kd;
    cfg.pid_shift = shift;
    cfg.motor_speed = motor_speed;
    
    return cfg;
}


ConfigSweeper create_no_variance_sweeper(void)
{
    ConfigSweeper sweeper;

    sweeper.motor_speed = {150u};
    sweeper.motor_speed_scale = {1.0};
    sweeper.dt = {0.01};
    sweeper.motor1_variance = {0.0};
    sweeper.motor2_variance = {0.0};
    sweeper.slip_factor = {1.0};
    sweeper.wheel_circumference_scale = {1.0};
    sweeper.wheel_base_scale = {1.0};
    sweeper.kp = {0};
    sweeper.kd = {0};
    sweeper.pid_shift = {0};

    return sweeper;
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
TEST(RotationTests, ConfigSweeperProducesFirstValue)
{
    ConfigSweeper sweeper{create_no_variance_sweeper()};

    CHECK(sweeper.next());

    auto cfg{sweeper.value()};

    CHECK_EQUAL(150, cfg.motor_speed);
    CHECK_EQUAL(0, cfg.kp);
    CHECK_EQUAL(0, cfg.kd);
    CHECK_EQUAL(0, cfg.pid_shift);
}

TEST(RotationTests, ConfigSweeperIteratesAllCombinations)
{
    ConfigSweeper sweeper{create_no_variance_sweeper()};
    sweeper.motor_speed = {100, 200};
    sweeper.kp = {1, 2};

    int count{0};

    while (sweeper.next()) {
        sweeper.value();
        count++;
    }

    CHECK_EQUAL(4, count); /* 2 motor_speed * 2 kp */
}

TEST(RotationTests, ConfigSweeperStopsAtEnd)
{
    ConfigSweeper sweeper{create_no_variance_sweeper()};

    CHECK(sweeper.next());
    CHECK_FALSE(sweeper.next());
}

TEST(RotationTests, ConfigSweeperOrderIsStable)
{
    ConfigSweeper sweeper{create_no_variance_sweeper()};
    sweeper.kp = {1, 2};
    sweeper.kd = {10, 20};

    std::vector<std::pair<int,int>> seen;

    while (sweeper.next()) {
        auto cfg{sweeper.value()};
        seen.emplace_back(cfg.kp, cfg.kd);
    }

    CHECK_EQUAL(4, seen.size());

    /* Expected order: kp outer, kd inner */
    CHECK(seen[0] == std::make_pair(1,10));
    CHECK(seen[1] == std::make_pair(1,20));
    CHECK(seen[2] == std::make_pair(2,10));
    CHECK(seen[3] == std::make_pair(2,20));
}

TEST(RotationTests, SimulationProducesValidResult)
{
    Config cfg{create_no_variance_config()};

    auto r{run_simulation(test_maze, cfg, M_PI / 2)};

    CHECK(r.total_time >= 0.0);
    CHECK(r.final_angle_error >= 0.0);
    CHECK(r.total_translation >= 0.0);
}

TEST(RotationTests, SimulationFailsWhenDtIsZero)
{
    Config cfg{create_no_variance_config()};
    cfg.dt = 0.0;

    auto r{run_simulation(test_maze, cfg, M_PI / 2)};

    CHECK(r.timeout);
}

TEST(RotationTests, PositiveAndNegativeAnglesProduceSameAngleAndTranslationError)
{
    Config cfg{create_no_variance_config()};

    auto r1{run_simulation(test_maze, cfg,  M_PI / 2)};
    auto r2{run_simulation(test_maze, cfg, -M_PI / 2)};

    CHECK_FALSE(r1.timeout);
    CHECK_FALSE(r2.timeout);
    CHECK(r1.total_translation == r2.total_translation);
    CHECK(r1.final_angle_error == r2.final_angle_error);
}

TEST(RotationTests, LargerAngleTakesMoreTime)
{
    Config cfg{create_no_variance_config()};

    auto small{run_simulation(test_maze, cfg, M_PI / 4)};
    auto large{run_simulation(test_maze, cfg, M_PI / 2)};

    CHECK(large.total_time >= small.total_time);
}

TEST(RotationTests, SimulationCanDetectCollision)
{
    std::vector<std::string> ascii{
        "+-+-+",
        "|S  |",
        "+-+-+"
    };

    maze::Maze maze{maze::build_maze_from_ascii(ascii, 0.0)};

    Config cfg{create_no_variance_config()};
    cfg.motor_speed = 255u;
    cfg.motor1_variance = -1;

    auto r{run_simulation(test_maze, cfg, M_PI)};

    CHECK(r.collision);
}

TEST(RotationTests, NoTranslationAndAngleErrorForPerfectTestVariables)
{
    std::vector<std::string> ascii{
        "+-+",
        "|S|",
        "+-+"
    };
    maze::Maze maze{maze::build_maze_from_ascii(ascii, 0.0)};

    /* slow movement, tiny dt, and no motor variances */
    Config cfg{create_no_variance_config()};
    cfg.motor_speed = 100;
    cfg.dt = 0.001;

    auto r{run_simulation(test_maze, cfg, M_PI)};

    /* 1% of a circle, or 3.6 degrees */
    constexpr double ROTATION_TOLERANCE{(2 * M_PI) * 0.01};

    CHECK_FALSE(r.timeout);
    DOUBLES_EQUAL(0.0, r.final_angle_error, ROTATION_TOLERANCE);
    DOUBLES_EQUAL(0.0, r.total_translation, 0.01);
}

TEST(RotationTests, ComputeResultsMetricsProducesCorrectStats)
{
    std::vector<Result> results{
        {0,1, 10, false, false},
        {0,2, 20, false, false},
        {0,3, 30, false, false}
    };

    auto s{compute_results_metrics(results)};

    DOUBLES_EQUAL(20.0, s.translation_stats.mean, FLOAT_TOLERANCE);
    CHECK_EQUAL(10.0, s.translation_stats.min);
    CHECK_EQUAL(30.0, s.translation_stats.max);
}

TEST(RotationTests, ComputeResultsMetricsComputesRates)
{
    std::vector<Result> results{
        {0, 0, 0, false, false},
        {0, 0, 0, true, false},
        {0, 0, 0, false, true},
        {0, 0, 0, true, true}
    };

    auto s{compute_results_metrics(results)};

    DOUBLES_EQUAL(0.5, s.timeout_rate, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(0.5, s.collision_rate, FLOAT_TOLERANCE);
}

TEST(RotationTests, ComputeResultsMetricsHandlesEmptyInput)
{
    std::vector<Result> results{};

    auto s{compute_results_metrics(results)};

    DOUBLES_EQUAL(0.0, s.timeout_rate, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(0.0, s.collision_rate, FLOAT_TOLERANCE);
}

TEST(RotationTests, DerivativeTermAffectsStability)
{
    Config cfg{create_no_variance_config()};
    cfg.motor1_variance = -0.2;
    cfg.kp = 2000;
    cfg.pid_shift = 8;

    Config no_d{cfg};
    Config with_d{cfg};
    with_d.kd = 1000;

    auto r1{run_simulation(test_maze, no_d,  M_PI / 2)};
    auto r2{run_simulation(test_maze, with_d,M_PI / 2)};

    CHECK((r1.final_angle_error != r2.final_angle_error)
       || (r1.total_translation != r2.total_translation));
}

TEST(RotationTests, PDImprovesAccuracyOverNoControl)
{
    Config cfg{create_no_variance_config()};
    cfg.motor1_variance = -0.2;
    cfg.pid_shift = 8;

    Config no_control{cfg};
    Config pd_control{cfg};
    pd_control.kp = 2000;
    pd_control.kd = 1000;

    auto r1{run_simulation(test_maze, no_control, M_PI / 2)};
    auto r2{run_simulation(test_maze, pd_control, M_PI / 2)};

    CHECK(r2.final_angle_error <= r1.final_angle_error);
}

TEST(RotationTests, PidShiftAffectsControlStrength)
{
    Config cfg{create_no_variance_config()};
    cfg.motor1_variance = -0.2;
    cfg.kp = 2000;
    cfg.kd = 1000;

    Config strong{cfg};
    strong.pid_shift = 2;
    Config weak{cfg};
    weak.pid_shift = 8;

    auto r1{run_simulation(test_maze, strong, M_PI / 2)};
    auto r2{run_simulation(test_maze, weak,   M_PI / 2)};

    CHECK((r1.total_time != r2.total_time)
       || (r1.final_angle_error != r2.final_angle_error));
}

TEST(RotationTests, BuildCandidatesGroupsAndComputesStats)
{
    std::vector<Trial> trials{
        {create_config_custom_pid_and_speed(1,2,3), {10, 1.0, 100, false, false}},
        {create_config_custom_pid_and_speed(1,2,3), {20, 2.0, 200, false, false}},
        {create_config_custom_pid_and_speed(4,5,6), {30, 3.0, 300, true,  true}}
    };

    auto candidates{build_candidates(trials)};

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

TEST(RotationTests, RunMinimalSampleSimulation)
{
    ConfigSweeper sweeper;

    sweeper.motor_speed_scale = {1.0};
    sweeper.dt = {0.001};
    sweeper.motor1_variance = {0.0};
    sweeper.motor2_variance = {0.0};
    sweeper.slip_factor = {1.0};
    sweeper.wheel_circumference_scale = {1.0};
    sweeper.wheel_base_scale = {1.0};
    
    sweeper.motor_speed = {100};
    sweeper.kp = {0};
    sweeper.kd = {0};
    sweeper.pid_shift = {8};

    const std::string filename {"test_minimal_output.txt"};

    run_full_rotation_experiment(filename, M_PI / 2, sweeper);
}

TEST(RotationTests, CandidateKeyOrderingUsesMotorSpeed)
{
    CandidateKey a{1, 2, 3, 100};
    CandidateKey b{1, 2, 3, 200};

    CHECK(a < b);
    CHECK_FALSE(b < a);
}

TEST(RotationTests, CandidateKeyDifferentMotorSpeedNotEquivalent)
{
    CandidateKey a{1, 2, 3, 100};
    CandidateKey b{1, 2, 3, 200};

    /* Equivalent in std::map means !(a < b) && !(b < a) */
    bool equivalent {!(a < b) && !(b < a)};

    CHECK_FALSE(equivalent);
}

TEST(RotationTests, CandidateKeyMapSeparatesDifferentSpeeds)
{
    std::map<CandidateKey, int> m;

    CandidateKey k1{1, 2, 3, 100};
    CandidateKey k2{1, 2, 3, 200};

    m[k1] = 1;
    m[k2] = 2;

    CHECK_EQUAL(2, m.size());
}

TEST(RotationTests, BuildCandidatesSeparatesMotorSpeed)
{
    std::vector<Trial> trials{
        {create_config_custom_pid_and_speed(1,2,3,100), {10,1,100,false,false}},
        {create_config_custom_pid_and_speed(1,2,3,200), {20,2,200,false,false}}
    };

    auto candidates{build_candidates(trials)};

    CHECK_EQUAL(2, candidates.size());
}

TEST(RotationTests, ParetoFrontRemovesDominatedCandidate)
{
    Candidate a{
        {1,2,3,100},
        {{1,0,1,1}, {1,0,1,1}, {1,0,1,1}, 0.0, 0.0}
    };

    Candidate b{
        {1,2,3,100},
        {{2,0,2,2}, {2,0,2,2}, {2,0,2,2}, 0.5, 0.5}
    };

    std::vector<Candidate> v{a, b};

    auto front{compute_pareto_front(v)};

    CHECK_EQUAL(1, front.size());
    CHECK(front[0].results_metrics.time_stats.mean == a.results_metrics.time_stats.mean);
}

TEST(RotationTests, ParetoFrontKeepsNonDominatingCandidates)
{
    Candidate a{
        {1,2,3,100},
        {{1,0,1,1}, {10,0,10,10}, {1,0,1,1}, 0.0, 0.0}
    };

    Candidate b{
        {1,2,3,100},
        {{10,0,10,10}, {1,0,1,1}, {1,0,1,1}, 0.0, 0.0}
    };

    std::vector<Candidate> v{a, b};

    auto front{compute_pareto_front(v)};

    CHECK_EQUAL(2, front.size());
}

TEST(RotationTests, ParetoFrontKeepsIdenticalCandidates)
{
    Candidate a{
        {1,2,3,100},
        {{1,0,1,1}, {1,0,1,1}, {1,0,1,1}, 0.0, 0.0}
    };

    std::vector<Candidate> v{a, a};

    auto front{compute_pareto_front(v)};

    CHECK_EQUAL(2, front.size());
}

IGNORE_TEST(RotationTests, RunFullSimulationAndWriteResultsToFile)
{
    ConfigSweeper sweeper;

    sweeper.motor_speed_scale = simulation_common::generate_sweep_values(0.9, 1.1, 3);
    sweeper.dt = simulation_common::generate_sweep_values(0.01, 0.5, 3);
    sweeper.motor1_variance = simulation_common::generate_sweep_values(-0.1, 0.1, 3);
    sweeper.motor2_variance = simulation_common::generate_sweep_values(-0.1, 0.1, 3);
    sweeper.slip_factor = simulation_common::generate_sweep_values(0.9, 1.1, 3);
    sweeper.wheel_circumference_scale = simulation_common::generate_sweep_values(0.95, 1.05, 3);
    sweeper.wheel_base_scale = simulation_common::generate_sweep_values(0.95, 1.05, 3);
    
    sweeper.motor_speed = simulation_common::generate_sweep_values<uint8_t>(120, 220, 6);
    sweeper.kp = simulation_common::generate_sweep_values(0, 4000, 21);
    sweeper.kd = simulation_common::generate_sweep_values(0, 2000, 21);
    sweeper.pid_shift = {8};

    run_full_rotation_experiment("test_full_output.txt", M_PI / 2, sweeper);
}
