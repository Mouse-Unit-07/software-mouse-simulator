/*================================ FILE INFO =================================*/
/* Filename           : test_obstacle.cpp                                     */
/*                                                                            */
/* Test implementation for obstacle.cpp                                       */
/*                                                                            */
/*============================================================================*/

/*============================================================================*/
/*                               Include Files                                */
/*============================================================================*/
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
TEST(ObstacleTests, ParameterizedConstructorInitializesAllFields)
{
    geometry::Point test_center {0.0, 0.0};
    geometry::RectangularHitbox test_hitbox {test_center, obstacle::OFFICIAL_POST_SIZE, obstacle::OFFICIAL_POST_SIZE};
    obstacle::Post test_post {test_center, 0.0, 0.0};
    CHECK(test_post.hitbox == test_hitbox);
    
    double horizontal_adjustment {1.0};
    double vertical_adjustment {1.0};
    geometry::RectangularHitbox test_hitbox_2 {
        test_center, 
        obstacle::OFFICIAL_POST_SIZE + horizontal_adjustment, 
        obstacle::OFFICIAL_POST_SIZE + vertical_adjustment
    };
    obstacle::Post test_post_2 {test_center, horizontal_adjustment, vertical_adjustment};
    CHECK(test_post_2.hitbox == test_hitbox_2);
}
