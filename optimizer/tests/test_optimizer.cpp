/*================================ FILE INFO =================================*/
/* Filename           : test_optimizer.cpp                                    */
/*                                                                            */
/* Test implementation for optimizer.cpp                                      */
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

}

#include <cmath>
#include <cstdint>
#include <vector>
#include <string>
#include <optional>
#include <functional>
#include "point.hpp"
#include "ray.hpp"
#include "rectangular_hitbox.hpp"
#include "mouse.hpp"
#include "maze.hpp"
#include "optimizer.hpp"

#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>

using namespace optimizer;

/*============================================================================*/
/*                             Public Definitions                             */
/*============================================================================*/
constexpr double FLOAT_TOLERANCE {1e-6};

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
TEST(OptimizerTests, SweepGeneratesCorrectNumberOfResults)
{
    std::vector<std::string> ascii
    {
        "+-+",
        "|S|",
        "+-+"
    };
    maze::Maze maze {maze::build_maze_from_ascii(ascii, 0.0)};

    std::vector<optimizer::SweepParam> params
    {
        {"motor_speed", 100, 200, 2},
        {"motor_speed_scale", 1.0, 1.0, 1},
        {"dt", 0.01, 0.02, 2},

        {"motor1_variance", 0.0, 0.0, 1},
        {"motor2_variance", 0.0, 0.0, 1},
        {"slip_factor", 1.0, 1.0, 1},
        {"wheel_circumference_scale", 1.0, 1.0, 1},
        {"wheel_base_scale", 1.0, 1.0, 1}
    };

    auto sim_fn = [&](const std::vector<double>& vals) -> optimizer::RotationResult {
        auto cfg {optimizer::build_rotation_config(vals)};
        return optimizer::run_rotation_simulation(maze, cfg, M_PI / 2);
    };
    
    auto results {optimizer::run_parameter_sweep<optimizer::RotationResult>(params, sim_fn)};

    /* expected = 2 * 1 * 2 * 1 * 1 * 1 * 1 * 1 = 4 */
    CHECK_EQUAL(4, results.size());
}


TEST(OptimizerTests, SweepSingleStepProducesSingleResult)
{
    std::vector<std::string> ascii
    {
        "+-+",
        "|S|",
        "+-+"
    };
    maze::Maze maze {maze::build_maze_from_ascii(ascii, 0.0)};

    std::vector<optimizer::SweepParam> params
    {
        {"motor_speed", 150, 150, 1},
        {"motor_speed_scale", 1.0, 1.0, 1},
        {"dt", 0.01, 0.01, 1},

        {"motor1_variance", 0.0, 0.0, 1},
        {"motor2_variance", 0.0, 0.0, 1},
        {"slip_factor", 1.0, 1.0, 1},
        {"wheel_circumference_scale", 1.0, 1.0, 1},
        {"wheel_base_scale", 1.0, 1.0, 1}
    };

    auto sim_fn = [&](const std::vector<double>& vals) -> optimizer::RotationResult {
        auto cfg {optimizer::build_rotation_config(vals)};
        return optimizer::run_rotation_simulation(maze, cfg, M_PI / 2);
    };
    
    auto results {optimizer::run_parameter_sweep<optimizer::RotationResult>(params, sim_fn)};

    CHECK_EQUAL(1, results.size());
}

TEST(OptimizerTests, ResultsContainValidValues)
{
    std::vector<std::string> ascii
    {
        "+-+",
        "|S|",
        "+-+"
    };
    maze::Maze maze {maze::build_maze_from_ascii(ascii, 0.0)};

    std::vector<optimizer::SweepParam> params
    {
        {"motor_speed", 150, 150, 1},
        {"motor_speed_scale", 1.0, 1.0, 1},
        {"dt", 0.01, 0.01, 1},

        {"motor1_variance", 0.0, 0.0, 1},
        {"motor2_variance", 0.0, 0.0, 1},
        {"slip_factor", 1.0, 1.0, 1},
        {"wheel_circumference_scale", 1.0, 1.0, 1},
        {"wheel_base_scale", 1.0, 1.0, 1}
    };

    auto sim_fn = [&](const std::vector<double>& vals) -> optimizer::RotationResult {
        auto cfg {optimizer::build_rotation_config(vals)};
        return optimizer::run_rotation_simulation(maze, cfg, M_PI / 2);
    };
    
    auto results {optimizer::run_parameter_sweep<optimizer::RotationResult>(params, sim_fn)};

    const auto& r {results[0].second};

    CHECK(r.total_time >= 0.0);
    CHECK(r.final_angle_error >= 0.0);
    CHECK(r.total_translation >= 0.0);
}

TEST(OptimizerTests, SweepDetectsSimulationFailureWhenDtTooSmall)
{
    std::vector<std::string> ascii
    {
        "+-+",
        "|S|",
        "+-+"
    };
    maze::Maze maze {maze::build_maze_from_ascii(ascii, 0.0)};

    std::vector<optimizer::SweepParam> params
    {
        {"motor_speed", 150, 150, 1},
        {"motor_speed_scale", 1.0, 1.0, 1},
        {"dt", 0.0, 0.0, 1},  /* causes encoder not to change */

        {"motor1_variance", 0.0, 0.0, 1},
        {"motor2_variance", 0.0, 0.0, 1},
        {"slip_factor", 1.0, 1.0, 1},
        {"wheel_circumference_scale", 1.0, 1.0, 1},
        {"wheel_base_scale", 1.0, 1.0, 1}
    };

    auto sim_fn = [&](const std::vector<double>& vals) -> optimizer::RotationResult {
        auto cfg {optimizer::build_rotation_config(vals)};
        return optimizer::run_rotation_simulation(maze, cfg, M_PI / 2);
    };
    
    auto results {optimizer::run_parameter_sweep<optimizer::RotationResult>(params, sim_fn)};

    CHECK(results[0].second.simulation_failed);
}

TEST(OptimizerTests, SweepProducesDifferentResultsForDifferentMotorSpeeds)
{
    std::vector<std::string> ascii
    {
        "+-+",
        "|S|",
        "+-+"
    };
    maze::Maze maze {maze::build_maze_from_ascii(ascii, 0.0)};

    std::vector<optimizer::SweepParam> params
    {
        {"motor_speed", 100, 200, 2},
        {"motor_speed_scale", 1.0, 1.0, 1},
        {"dt", 0.01, 0.01, 1},

        {"motor1_variance", 0.0, 0.0, 1},
        {"motor2_variance", 0.0, 0.0, 1},
        {"slip_factor", 1.0, 1.0, 1},
        {"wheel_circumference_scale", 1.0, 1.0, 1},
        {"wheel_base_scale", 1.0, 1.0, 1}
    };

    auto sim_fn = [&](const std::vector<double>& vals) -> optimizer::RotationResult {
        auto cfg {optimizer::build_rotation_config(vals)};
        return optimizer::run_rotation_simulation(maze, cfg, M_PI / 2);
    };
    
    auto results {optimizer::run_parameter_sweep<optimizer::RotationResult>(params, sim_fn)};

    CHECK(results.size() == 2);

    /* Expect different time or translation */
    CHECK((results[0].second.total_time != results[1].second.total_time)
        || (results[0].second.total_translation != results[1].second.total_translation));
}

TEST(OptimizerTests, AnalyzeRotationResultsComputesBasicStats)
{
    std::vector<std::pair<std::vector<double>, RotationResult>> trials
    {
        {{1.0}, {0.0, 1.0, 10.0, false, false}},
        {{2.0}, {0.0, 2.0, 20.0, false, false}},
        {{3.0}, {0.0, 3.0, 30.0, false, false}}
    };

    auto summary {analyze_rotation_results(trials)};

    DOUBLES_EQUAL(20.0, summary.translation_stats.mean, FLOAT_TOLERANCE);
    CHECK(summary.translation_stats.min == 10.0);
    CHECK(summary.translation_stats.max == 30.0);

    DOUBLES_EQUAL(2.0, summary.angle_error_stats.mean, FLOAT_TOLERANCE);

    double expected_stddev {std::sqrt(66.6666667)};
    DOUBLES_EQUAL(expected_stddev, summary.translation_stats.stddev, FLOAT_TOLERANCE);
}

TEST(OptimizerTests, AnalyzeRotationResultsComputesFailureAndCollisionRates)
{
    std::vector<std::pair<std::vector<double>, RotationResult>> trials
    {
        {{}, {0, 0, 0, false, false}},
        {{}, {0, 0, 0, true, false}},
        {{}, {0, 0, 0, false, true}},
        {{}, {0, 0, 0, true, true}}
    };

    auto summary {analyze_rotation_results(trials)};

    DOUBLES_EQUAL(0.5, summary.failure_rate, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(0.5, summary.collision_rate, FLOAT_TOLERANCE);
}

TEST(OptimizerTests, ParameterImpactSplitsLowHighCorrectly)
{
    std::vector<SweepParam> params
    {
        {"test_param", 0.0, 10.0, 2}
    };

    std::vector<std::pair<std::vector<double>, RotationResult>> trials
    {
        {{1.0}, {0, 0, 0, false, false}}, // low
        {{2.0}, {0, 0, 0, false, true}},  // low (failure)
        {{8.0}, {0, 0, 0, true, false}},  // high (collision)
        {{9.0}, {0, 0, 0, false, false}}  // high
    };

    auto impacts {analyze_rotation_parameter_impact(params, trials)};

    CHECK_EQUAL(1, impacts.size());

    const auto& impact {impacts[0]};

    DOUBLES_EQUAL(0.5, impact.failure_rate_low, FLOAT_TOLERANCE);   /* 1/2 */
    DOUBLES_EQUAL(0.0, impact.failure_rate_high, FLOAT_TOLERANCE);  /* 0/2 */

    DOUBLES_EQUAL(0.0, impact.collision_rate_low, FLOAT_TOLERANCE); /* 0/2 */
    DOUBLES_EQUAL(0.5, impact.collision_rate_high, FLOAT_TOLERANCE);/* 1/2 */
}

TEST(OptimizerTests, ParameterImpactDetectsPositiveCorrelation)
{
    std::vector<SweepParam> params
    {
        {"p", 0, 10, 3}
    };

    std::vector<std::pair<std::vector<double>, RotationResult>> trials
    {
        {{1.0}, {0, 1.0, 10.0, false, false}},
        {{2.0}, {0, 2.0, 20.0, false, false}},
        {{3.0}, {0, 3.0, 30.0, false, false}}
    };

    auto impacts {analyze_rotation_parameter_impact(params, trials)};

    DOUBLES_EQUAL(1.0, impacts[0].correlation_translation, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(1.0, impacts[0].correlation_angle_error, FLOAT_TOLERANCE);
}

TEST(OptimizerTests, ParameterImpactDetectsNegativeCorrelation)
{
    std::vector<SweepParam> params
    {
        {"p", 0, 10, 3}
    };

    std::vector<std::pair<std::vector<double>, RotationResult>> trials
    {
        {{1.0}, {0, 3.0, 30.0, false, false}},
        {{2.0}, {0, 2.0, 20.0, false, false}},
        {{3.0}, {0, 1.0, 10.0, false, false}}
    };

    auto impacts {analyze_rotation_parameter_impact(params, trials)};

    DOUBLES_EQUAL(-1.0, impacts[0].correlation_translation, FLOAT_TOLERANCE);
}

TEST(OptimizerTests, ParameterImpactHandlesZeroVariance)
{
    std::vector<SweepParam> params
    {
        {"p", 0, 10, 3}
    };

    std::vector<std::pair<std::vector<double>, RotationResult>> trials
    {
        {{1.0}, {0, 1.0, 10.0, false, false}},
        {{1.0}, {0, 2.0, 20.0, false, false}},
        {{1.0}, {0, 3.0, 30.0, false, false}}
    };

    auto impacts {analyze_rotation_parameter_impact(params, trials)};

    /* x variance = 0 -> correlation should safely return ~0 */
    DOUBLES_EQUAL(0.0, impacts[0].correlation_translation, FLOAT_TOLERANCE);
}

TEST(OptimizerTests, ParameterImpactHandlesMultipleParameters)
{
    std::vector<SweepParam> params
    {
        {"p1", 0, 10, 2},
        {"p2", 0, 5, 2}
    };

    std::vector<std::pair<std::vector<double>, RotationResult>> trials
    {
        {{1.0, 1.0}, {0, 1.0, 10.0, false, false}},
        {{2.0, 1.0}, {0, 2.0, 20.0, false, false}},
        {{1.0, 5.0}, {0, 3.0, 30.0, false, false}},
        {{2.0, 5.0}, {0, 4.0, 40.0, false, false}}
    };

    auto impacts {analyze_rotation_parameter_impact(params, trials)};

    CHECK_EQUAL(2, impacts.size());

    CHECK(impacts[0].name == "p1");
    CHECK(impacts[1].name == "p2");
}

TEST(OptimizerTests, ParameterImpactMedianSplitHandlesEqualValues)
{
    std::vector<SweepParam> params
    {
        {"p", 0, 10, 4}
    };

    std::vector<std::pair<std::vector<double>, RotationResult>> trials
    {
        {{1.0}, {0, 0, 0, false, false}},
        {{2.0}, {0, 0, 0, false, true}},   // failure
        {{2.0}, {0, 0, 0, false, false}},  // equal to mid candidate
        {{9.0}, {0, 0, 0, true, false}}
    };

    auto impacts {analyze_rotation_parameter_impact(params, trials)};

    /* Not asserting exact rates- just ensuring no crash and valid bounds */
    CHECK(impacts[0].failure_rate_low >= 0.0);
    CHECK(impacts[0].failure_rate_high >= 0.0);
}
