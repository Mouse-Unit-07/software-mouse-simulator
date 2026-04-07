/*================================ FILE INFO =================================*/
/* Filename           : test_move_forward.cpp                                 */
/*                                                                            */
/* Test implementation for move_forward.cpp                                   */
/*                                                                            */
/*============================================================================*/

/*============================================================================*/
/*                               Include Files                                */
/*============================================================================*/
extern "C"
{

}

#include <cstdint>
#include <vector>
#include <string>
#include <map>
#include "simulation_common.hpp"
#include "move_forward.hpp"

#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>

using namespace move_forward;

/*============================================================================*/
/*                             Public Definitions                             */
/*============================================================================*/
constexpr double FLOAT_TOLERANCE {1e-6};

ConfigSweeper create_no_variance_sweeper(void)
{
    ConfigSweeper sweeper;

    sweeper.dt = {0.01};
    sweeper.motor_speed_scale = {1.0};
    sweeper.motor1_variance = {0.0};
    sweeper.motor2_variance = {0.0};
    sweeper.slip_factor = {1.0};
    sweeper.wheel_circumference_scale = {1.0};
    sweeper.wheel_base_scale = {1.0};
    sweeper.maze_size_scale = {1.0};
    sweeper.ir_reading_scale = {1.0};
    sweeper.mouse_angle = {0.0};
    sweeper.horizontal_position_variance = {0.0};
    sweeper.vertical_position_variance = {0.0};
    sweeper.wall_detection_threshold = {840};
    sweeper.wall_detection_start_percent = {0.32};
    sweeper.wall_detection_window_size_percent = {0.2};

    sweeper.motor_speed = {150u};
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
TEST_GROUP(MoveForwardTests)
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
TEST(MoveForwardTests, ConfigSweeperProducesFirstValue)
{
    ConfigSweeper sweeper{create_no_variance_sweeper()};

    CHECK(sweeper.next());

    auto cfg{sweeper.value()};

    CHECK_EQUAL(840, cfg.wall_detection_threshold);
    DOUBLES_EQUAL(0.32, cfg.wall_detection_start_percent, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(0.2, cfg.wall_detection_window_size_percent, FLOAT_TOLERANCE);
}

TEST(MoveForwardTests, ConfigSweeperIteratesAllCombinations)
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

TEST(MoveForwardTests, ConfigSweeperStopsAtEnd)
{
    ConfigSweeper sweeper{create_no_variance_sweeper()};

    CHECK(sweeper.next());
    CHECK_FALSE(sweeper.next());
}

TEST(MoveForwardTests, ConfigSweeperOrderIsStable)
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
    CHECK(seen.at(0) == std::make_pair(1,10));
    CHECK(seen.at(1) == std::make_pair(1,20));
    CHECK(seen.at(2) == std::make_pair(2,10));
    CHECK(seen.at(3) == std::make_pair(2,20));
}
