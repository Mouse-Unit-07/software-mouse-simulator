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
geometry::Point test_center {0.0, 0.0};
geometry::RectangularHitbox test_hitbox {test_center, obstacle::OFFICIAL_POST_SIZE, obstacle::OFFICIAL_POST_SIZE};
obstacle::Post test_post {test_center, 0.0, 0.0};

void initialize_test_variables(void)
{
    test_center = geometry::Point{0.0, 0.0};
    test_hitbox = geometry::RectangularHitbox{test_center, obstacle::OFFICIAL_POST_SIZE, obstacle::OFFICIAL_POST_SIZE};
    test_post = obstacle::Post{test_center, 0.0, 0.0};
}

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
        initialize_test_variables();
    }

    void teardown() override
    {
        initialize_test_variables();
    }
};

/*============================================================================*/
/*                                    Tests                                   */
/*============================================================================*/
TEST(ObstacleTests, ParameterizedConstructorInitializesAllFields)
{
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

TEST(ObstacleTests, TranslateModifiesHitbox)
{
    double test_dx = 1.0;
    double test_dy = 2.0;
    test_post.translate(test_dx, test_dy);
    test_hitbox.translate(test_dx, test_dy);
    CHECK(test_post.hitbox == test_hitbox);
}
