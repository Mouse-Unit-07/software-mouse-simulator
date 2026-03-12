/*================================ FILE INFO =================================*/
/* Filename           : test_obstacle.cpp                                     */
/*                                                                            */
/* Test implementation for obstacle.cpp                                       */
/*                                                                            */
/*============================================================================*/

/*============================================================================*/
/*                               Include Files                                */
/*============================================================================*/
#include <cmath>
#include "point.hpp"
#include "rectangular_hitbox.hpp"
#include "obstacle.hpp"
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
TEST_GROUP(ObstacleTests)
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
TEST(ObstacleTests, PostWithSpecifiedAdjustmentsCreated)
{
    double adjustment {2.0};
    geometry::RectangularHitbox test_hitbox {
        obstacle::create_post(geometry::Point{0.0, 0.0}, adjustment, adjustment)
    };
    CHECK(test_hitbox.horizontal_size == (obstacle::OFFICIAL_POST_SIZE + adjustment));
    CHECK(test_hitbox.vertical_size == (obstacle::OFFICIAL_POST_SIZE + adjustment));
}
