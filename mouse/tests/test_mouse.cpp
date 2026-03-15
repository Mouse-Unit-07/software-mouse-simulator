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
    mouse::Mouse test_mouse;
    DOUBLES_EQUAL(0.0, test_mouse.hitbox.center.x, 1e-6);
    DOUBLES_EQUAL(0.0, test_mouse.hitbox.center.y, 1e-6);
}

TEST(MouseTests, TranslateMovesHitboxAndRays)
{
    mouse::Mouse test_mouse;

    double test_dx = 10.0;
    double test_dy = 5.0;
    geometry::Point center = test_mouse.hitbox.center;
    geometry::Point ir_1_origin = test_mouse.ir_1_sensor.origin;
    geometry::Point ir_2_origin = test_mouse.ir_2_sensor.origin;
    geometry::Point ir_3_origin = test_mouse.ir_3_sensor.origin;
    geometry::Point ir_4_origin = test_mouse.ir_4_sensor.origin;
    test_mouse.translate(test_dx, test_dy);

    center.translate(test_dx, test_dy);
    ir_1_origin.translate(test_dx, test_dy);
    ir_2_origin.translate(test_dx, test_dy);
    ir_3_origin.translate(test_dx, test_dy);
    ir_4_origin.translate(test_dx, test_dy);
    CHECK(test_mouse.hitbox.center == center);
    CHECK(test_mouse.ir_1_sensor.origin == ir_1_origin);
    CHECK(test_mouse.ir_2_sensor.origin == ir_2_origin);
    CHECK(test_mouse.ir_3_sensor.origin == ir_3_origin);
    CHECK(test_mouse.ir_4_sensor.origin == ir_4_origin);
}
