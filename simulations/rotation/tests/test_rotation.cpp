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
    std::vector<double> v {1, 2, 3, 4, 5, 6, 7, 8};

    auto cfg {build_rotation_config(v)};

    CHECK_EQUAL(1, cfg.motor_speed);
    CHECK_EQUAL(2, cfg.motor_speed_scale);
    CHECK_EQUAL(3, cfg.dt);
    CHECK_EQUAL(4, cfg.motor1_variance);
    CHECK_EQUAL(5, cfg.motor2_variance);
    CHECK_EQUAL(6, cfg.slip_factor);
    CHECK_EQUAL(7, cfg.wheel_circumference_scale);
    CHECK_EQUAL(8, cfg.wheel_base_scale);
}

TEST(RotationTests, SimulationProducesValidResult)
{
    RotationConfig cfg {150, 1.0, 0.01, 0, 0, 1, 1, 1};

    auto r {run_rotation_simulation(test_maze, cfg, M_PI / 2)};

    CHECK(r.total_time >= 0.0);
    CHECK(r.final_angle_error >= 0.0);
    CHECK(r.total_translation >= 0.0);
}

TEST(RotationTests, SimulationFailsWhenDtIsZero)
{
    RotationConfig cfg {150, 1.0, 0.0, 0, 0, 1, 1, 1};

    auto r {run_rotation_simulation(test_maze, cfg, M_PI / 2)};

    CHECK(r.simulation_failed);
}

TEST(RotationTests, PositiveAndNegativeAnglesProduceDifferentResults)
{
    RotationConfig cfg {150, 1.0, 0.01, 0, 0, 1, 1, 1};

    auto r1 {run_rotation_simulation(test_maze, cfg,  M_PI / 2)};
    auto r2 {run_rotation_simulation(test_maze, cfg, -M_PI / 2)};

    CHECK((r1.total_translation != r2.total_translation)
       || (r1.final_angle_error != r2.final_angle_error));
}

TEST(RotationTests, LargerAngleTakesMoreTime)
{
    RotationConfig cfg {150, 1.0, 0.01, 0, 0, 1, 1, 1};

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

    RotationConfig cfg {255, 1.0, 0.01, -1, 0, 1, 1, 1};

    auto r {run_rotation_simulation(test_maze, cfg, M_PI)};

    CHECK(r.collision);
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
