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
        
    }

    void teardown() override
    {
        
    }
};

/*============================================================================*/
/*                                    Tests                                   */
/*============================================================================*/
TEST(WallDetectionTests, BuildConfigMapsValuesCorrectly)
{
    std::vector<double> v {1, 2, 3, 4, 5, 6};

    auto cfg {build_config(v)};

    CHECK_EQUAL(1, cfg.maze_size_scale);
    CHECK_EQUAL(2, cfg.ir_reading_scale);
    CHECK_EQUAL(3, cfg.mouse_angle);
    CHECK_EQUAL(4, cfg.horizontal_position_variance);
    CHECK_EQUAL(5, cfg.total_steps);
    CHECK_EQUAL(6, cfg.reading_threshold);
}
