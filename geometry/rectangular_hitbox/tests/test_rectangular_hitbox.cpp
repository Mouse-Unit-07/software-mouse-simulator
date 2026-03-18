/*================================ FILE INFO =================================*/
/* Filename           : test_rectangular_hitbox.cpp                           */
/*                                                                            */
/* Test implementation for rectangular_hitbox.cpp                             */
/*                                                                            */
/*============================================================================*/

/*============================================================================*/
/*                               Include Files                                */
/*============================================================================*/
#include <cmath>
#include "point.hpp"
#include "rectangular_hitbox.hpp"
#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>

/*============================================================================*/
/*                             Public Definitions                             */
/*============================================================================*/
geometry::Point test_center {0.0, 0.0}; /* arbitrary center */
double test_horizontal_size {10.0}; /* arbitrary size */
double test_vertical_size {10.0}; /* arbitrary size */
geometry::RectangularHitbox test_hitbox {test_center, test_horizontal_size, test_vertical_size};
geometry::Point test_top_right {test_center.x + (test_horizontal_size / 2), test_center.y + (test_vertical_size / 2)};
geometry::Point test_top_left {test_center.x - (test_horizontal_size / 2), test_center.y + (test_vertical_size / 2)};
geometry::Point test_bottom_left {test_center.x - (test_horizontal_size / 2), test_center.y - (test_vertical_size / 2)};
geometry::Point test_bottom_right {test_center.x + (test_horizontal_size / 2), test_center.y - (test_vertical_size / 2)};

void initialize_test_variables(void)
{
    test_center = geometry::Point{0.0, 0.0};
    test_horizontal_size = 10.0;
    test_vertical_size = 10.0;
    test_hitbox = geometry::RectangularHitbox{test_center, test_horizontal_size, test_vertical_size};
    test_top_right = geometry::Point{test_center.x + (test_horizontal_size / 2), test_center.y + (test_vertical_size / 2)};
    test_top_left = geometry::Point{test_center.x - (test_horizontal_size / 2), test_center.y + (test_vertical_size / 2)};
    test_bottom_left = geometry::Point{test_center.x - (test_horizontal_size / 2), test_center.y - (test_vertical_size / 2)};
    test_bottom_right = geometry::Point{test_center.x + (test_horizontal_size / 2), test_center.y - (test_vertical_size / 2)};
}

/*============================================================================*/
/*                            Mock Implementations                            */
/*============================================================================*/
/* none */

/*============================================================================*/
/*                                 Test Group                                 */
/*============================================================================*/
TEST_GROUP(RectangularHitboxTests)
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
TEST(RectangularHitboxTests, ParameterizedConstructorInitializesAllFields)
{
    CHECK(test_hitbox.center == test_center);
    CHECK(test_hitbox.horizontal_size == test_horizontal_size);
    CHECK(test_hitbox.vertical_size == test_vertical_size);
    CHECK(test_top_right == test_hitbox.top_right);
    CHECK(test_bottom_right == test_hitbox.bottom_right);
    CHECK(test_bottom_left == test_hitbox.bottom_left);
    CHECK(test_top_left == test_hitbox.top_left);
}

TEST(RectangularHitboxTests, TranslateModifiesPoints)
{
    double test_dx {-10.0}; /* arbitrary translation */
    double test_dy {20.0};
    test_center.translate(test_dx, test_dy);
    test_top_right.translate(test_dx, test_dy);
    test_bottom_right.translate(test_dx, test_dy);
    test_bottom_left.translate(test_dx, test_dy);
    test_top_left.translate(test_dx, test_dy);
    test_hitbox.translate(test_dx, test_dy);

    CHECK(test_hitbox.center == test_center);
    CHECK(test_hitbox.top_right == test_top_right);
    CHECK(test_hitbox.bottom_right == test_bottom_right);
    CHECK(test_hitbox.bottom_left == test_bottom_left);
    CHECK(test_hitbox.top_left == test_top_left);
}

TEST(RectangularHitboxTests, RotateModifiesPoints)
{
    geometry::Point test_rotation_center {10.0, 10.0}; /* arbitrary center */
    double test_angle {M_PI / 2}; /* arbitrary rotation (90 deg counter clockwise) */
    test_center.rotate(test_rotation_center, test_angle);
    test_top_right.rotate(test_rotation_center, test_angle);
    test_bottom_right.rotate(test_rotation_center, test_angle);
    test_bottom_left.rotate(test_rotation_center, test_angle);
    test_top_left.rotate(test_rotation_center, test_angle);
    test_hitbox.rotate(test_rotation_center, test_angle);

    CHECK(test_hitbox.center == test_center);
    CHECK(test_hitbox.top_right == test_top_right);
    CHECK(test_hitbox.bottom_right == test_bottom_right);
    CHECK(test_hitbox.bottom_left == test_bottom_left);
    CHECK(test_hitbox.top_left == test_top_left);
}

TEST(RectangularHitboxTests, RotateModifiesAngle)
{
    constexpr double FLOAT_TOLERANCE {1e-6};

    test_hitbox.rotate(test_center, M_PI / 2);
    DOUBLES_EQUAL(M_PI, test_hitbox.angle_rad, FLOAT_TOLERANCE);

    test_hitbox.rotate(test_center, M_PI);
    DOUBLES_EQUAL(0, test_hitbox.angle_rad, FLOAT_TOLERANCE);

    test_hitbox.rotate(test_center, -M_PI / 2);
    DOUBLES_EQUAL(-M_PI / 2, test_hitbox.angle_rad, FLOAT_TOLERANCE);

    test_hitbox.rotate(test_center, -M_PI / 2);
    DOUBLES_EQUAL(M_PI, test_hitbox.angle_rad, FLOAT_TOLERANCE);

    test_hitbox.rotate(test_center, -M_PI / 2);
    DOUBLES_EQUAL(M_PI / 2, test_hitbox.angle_rad, FLOAT_TOLERANCE);
}

TEST(RectangularHitboxTests, EqualityOperatorOverloaded)
{
    geometry::RectangularHitbox test_hitbox_2 {test_center, test_horizontal_size, test_vertical_size};

    CHECK(test_hitbox_2 == test_hitbox);
}

TEST(RectangularHitboxTests, InequalityOperatorOverloaded)
{
    geometry::RectangularHitbox test_hitbox_2 {geometry::Point{1.0, 1.0}, test_horizontal_size, test_vertical_size};

    CHECK(test_hitbox_2 != test_hitbox);
}
