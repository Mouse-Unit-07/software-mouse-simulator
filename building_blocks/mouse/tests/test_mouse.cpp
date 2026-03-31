/*================================ FILE INFO =================================*/
/* Filename           : test_mouse.cpp                                        */
/*                                                                            */
/* Test implementation for mouse.cpp                                          */
/*                                                                            */
/*============================================================================*/

/*============================================================================*/
/*                               Include Files                                */
/*============================================================================*/
#include <cmath>
#include "point.hpp"
#include "ray.hpp"
#include "rectangular_hitbox.hpp"
#include "mouse.hpp"
#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>

using namespace mouse;

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
TEST_GROUP(MouseTests)
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
TEST(MouseTests, ConstructorInitializesCoordinatesToZero)
{
    Mouse test_mouse;
    DOUBLES_EQUAL(0.0, test_mouse.hitbox.center.x, 1e-6);
    DOUBLES_EQUAL(0.0, test_mouse.hitbox.center.y, 1e-6);
}

TEST(MouseTests, TranslateMovesHitboxAndRays)
{
    Mouse test_mouse;

    double test_dx{10.0};
    double test_dy{5.0};
    geometry::RectangularHitbox test_hitbox{test_mouse.hitbox};
    geometry::Ray ir_1{test_mouse.ir_1_sensor};
    geometry::Ray ir_2{test_mouse.ir_2_sensor};
    geometry::Ray ir_3{test_mouse.ir_3_sensor};
    geometry::Ray ir_4{test_mouse.ir_4_sensor};
    test_mouse.translate(test_dx, test_dy);

    test_hitbox.translate(test_dx, test_dy);
    ir_1.translate(test_dx, test_dy);
    ir_2.translate(test_dx, test_dy);
    ir_3.translate(test_dx, test_dy);
    ir_4.translate(test_dx, test_dy);
    CHECK(test_mouse.hitbox == test_hitbox);
    CHECK(test_mouse.ir_1_sensor == ir_1);
    CHECK(test_mouse.ir_2_sensor == ir_2);
    CHECK(test_mouse.ir_3_sensor == ir_3);
    CHECK(test_mouse.ir_4_sensor == ir_4);
}

TEST(MouseTests, RotateMovesHitboxAndRaysAboutHitboxCenter)
{
    Mouse test_mouse;

    double test_angle{M_PI / 4.0};
    geometry::Point center{test_mouse.hitbox.center};
    geometry::RectangularHitbox test_hitbox{test_mouse.hitbox};
    geometry::Ray ir_1{test_mouse.ir_1_sensor};
    geometry::Ray ir_2{test_mouse.ir_2_sensor};
    geometry::Ray ir_3{test_mouse.ir_3_sensor};
    geometry::Ray ir_4{test_mouse.ir_4_sensor};
    test_mouse.rotate(test_angle);

    test_hitbox.rotate(center, test_angle);
    ir_1.rotate(center, test_angle);
    ir_2.rotate(center, test_angle);
    ir_3.rotate(center, test_angle);
    ir_4.rotate(center, test_angle);
    CHECK(test_mouse.hitbox == test_hitbox);
    CHECK(test_mouse.ir_1_sensor == ir_1);
    CHECK(test_mouse.ir_2_sensor == ir_2);
    CHECK(test_mouse.ir_3_sensor == ir_3);
    CHECK(test_mouse.ir_4_sensor == ir_4);
}

TEST(MouseTests, EqualityAndInequalityOperatorOverloaded)
{
    Mouse test_mouse;
    Mouse test_mouse_2;
    Mouse test_mouse_3;
    CHECK(test_mouse == test_mouse_2);
    CHECK(test_mouse == test_mouse_3);

    test_mouse_2.translate(10.0, 20.0);
    test_mouse_3.rotate(M_PI / 2.0);

    CHECK(test_mouse != test_mouse_2);
    CHECK(test_mouse != test_mouse_3);
}
