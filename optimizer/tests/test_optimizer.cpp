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
        auto cfg = optimizer::build_rotation_config(vals);
        return optimizer::run_rotation_simulation(maze, cfg, M_PI / 2);
    };
    
    auto results = optimizer::run_parameter_sweep<optimizer::RotationResult>(params, sim_fn);

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
        auto cfg = optimizer::build_rotation_config(vals);
        return optimizer::run_rotation_simulation(maze, cfg, M_PI / 2);
    };
    
    auto results = optimizer::run_parameter_sweep<optimizer::RotationResult>(params, sim_fn);

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
        auto cfg = optimizer::build_rotation_config(vals);
        return optimizer::run_rotation_simulation(maze, cfg, M_PI / 2);
    };
    
    auto results = optimizer::run_parameter_sweep<optimizer::RotationResult>(params, sim_fn);

    const auto& r = results[0];

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
        auto cfg = optimizer::build_rotation_config(vals);
        return optimizer::run_rotation_simulation(maze, cfg, M_PI / 2);
    };
    
    auto results = optimizer::run_parameter_sweep<optimizer::RotationResult>(params, sim_fn);

    CHECK(results[0].simulation_failed);
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
        auto cfg = optimizer::build_rotation_config(vals);
        return optimizer::run_rotation_simulation(maze, cfg, M_PI / 2);
    };
    
    auto results = optimizer::run_parameter_sweep<optimizer::RotationResult>(params, sim_fn);

    CHECK(results.size() == 2);

    /* Expect different time or translation */
    CHECK((results[0].total_time != results[1].total_time) || (results[0].total_translation != results[1].total_translation));
}

