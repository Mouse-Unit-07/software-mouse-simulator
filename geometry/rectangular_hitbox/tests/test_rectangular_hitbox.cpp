/*================================ FILE INFO =================================*/
/* Filename           : test_rectangular_hitbox.cpp                           */
/*                                                                            */
/* Test implementation for rectangular_hitbox.cpp                             */
/*                                                                            */
/*============================================================================*/

/*============================================================================*/
/*                               Include Files                                */
/*============================================================================*/
#include "point.hpp"
#include "rectangular_hitbox.hpp"
#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>

/*============================================================================*/
/*                             Public Definitions                             */
/*============================================================================*/
/* none */

/*============================================================================*/
/*                            Mock Implementations                            */
/*============================================================================*/


/*============================================================================*/
/*                                 Test Group                                 */
/*============================================================================*/
TEST_GROUP(RectangularHitboxTests)
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
TEST(RectangularHitboxTests, ParameterizedConstructorInitializesAllFields)
{
    Point user_center {0.0, 0.0};
    double user_horizontal_size {10.0};
    double user_vertical_size {10.0};
    RectangularHitbox user_hitbox{Point{}, user_horizontal_size, user_vertical_size};
    CHECK(user_hitbox.center == user_center);
    CHECK(user_hitbox.horizontal_size == user_horizontal_size);
    CHECK(user_hitbox.vertical_size == user_vertical_size);
    
    Point user_edge_1{user_center.x + (user_horizontal_size / 2), user_center.y + (user_vertical_size / 2)};
    Point user_edge_2{user_center.x + (user_horizontal_size / 2), user_center.y - (user_vertical_size / 2)};
    Point user_edge_3{user_center.x - (user_horizontal_size / 2), user_center.y - (user_vertical_size / 2)};
    Point user_edge_4{user_center.x - (user_horizontal_size / 2), user_center.y + (user_vertical_size / 2)};
    CHECK(user_edge_1 == user_hitbox.edge_1);
    CHECK(user_edge_2 == user_hitbox.edge_2);
    CHECK(user_edge_3 == user_hitbox.edge_3);
    CHECK(user_edge_4 == user_hitbox.edge_4);
}

TEST(RectangularHitboxTests, TranslateModifiesPoints)
{
    Point user_center {0.0, 0.0};
    double user_horizontal_size {10.0};
    double user_vertical_size {10.0};
    RectangularHitbox user_hitbox{Point{}, user_horizontal_size, user_vertical_size};
    Point user_edge_1{user_center.x + (user_horizontal_size / 2), user_center.y + (user_vertical_size / 2)};
    Point user_edge_2{user_center.x + (user_horizontal_size / 2), user_center.y - (user_vertical_size / 2)};
    Point user_edge_3{user_center.x - (user_horizontal_size / 2), user_center.y - (user_vertical_size / 2)};
    Point user_edge_4{user_center.x - (user_horizontal_size / 2), user_center.y + (user_vertical_size / 2)};
    
    double user_dx {-10.0};
    double user_dy {20.0};
    user_center.translate(user_dx, user_dy);
    user_edge_1.translate(user_dx, user_dy);
    user_edge_2.translate(user_dx, user_dy);
    user_edge_3.translate(user_dx, user_dy);
    user_edge_4.translate(user_dx, user_dy);
    user_hitbox.translate(user_dx, user_dy);

    CHECK(user_hitbox.center == user_center);
    CHECK(user_hitbox.edge_1 == user_edge_1);
    CHECK(user_hitbox.edge_2 == user_edge_2);
    CHECK(user_hitbox.edge_3 == user_edge_3);
    CHECK(user_hitbox.edge_4 == user_edge_4);
}
