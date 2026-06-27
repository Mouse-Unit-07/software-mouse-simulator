/*================================ FILE INFO =================================*/
/* Filename           : test_point.cpp                                        */
/*                                                                            */
/* Test implementation for point.cpp                                          */
/*                                                                            */
/*============================================================================*/

/*============================================================================*/
/*                               Include Files                                */
/*============================================================================*/
#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>
#include <cmath>
#include "point.hpp"

using namespace geometry;

/*============================================================================*/
/*                             Public Definitions                             */
/*============================================================================*/
constexpr double FLOAT_TOLERANCE{1e-6};

/*============================================================================*/
/*                            Mock Implementations                            */
/*============================================================================*/
/* none */

/*============================================================================*/
/*                                 Test Group                                 */
/*============================================================================*/
TEST_GROUP(PointTests)
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
TEST(PointTests, CoordinatesAreZeroOnDefaultConstructor)
{
    Point p{};
    DOUBLES_EQUAL(0.0, p.x, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(0.0, p.y, FLOAT_TOLERANCE);
}

TEST(PointTests, CoordinatesMatchOnParameterizedConstructor)
{
    Point p{1.0, 3.0};
    DOUBLES_EQUAL(1.0, p.x, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(3.0, p.y, FLOAT_TOLERANCE);
}

TEST(PointTests, TranslateModifiesCoordinates)
{
    double user_x{1.0};
    double user_y{3.0};
    double user_dx{-10.0};
    double user_dy{20.0};
    Point p{user_x, user_y};
    p.translate(user_dx, user_dy);
    DOUBLES_EQUAL(user_x + user_dx, p.x, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(user_y + user_dy, p.y, FLOAT_TOLERANCE);
}

TEST(PointTests, RotateModifiesCoordinates)
{
    Point p{1.0, 1.0};
    p.rotate(Point{0.0, 0.0}, M_PI / 2); /* 90 deg counter clockwise */
    DOUBLES_EQUAL(-1.0, p.x, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(1.0, p.y, FLOAT_TOLERANCE);
}

TEST(PointTests, PointReturnsAfterFullRotation)
{
    double user_x{1.0};
    double user_y{1.0};
    Point p{user_x, user_y};
    for (int i{0}; i < 360; i++) {
        p.rotate(Point{0.0, 0.0}, M_PI / 180); /* 1 deg counter clockwise */
    }
    DOUBLES_EQUAL(user_x, p.x, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(user_y, p.y, FLOAT_TOLERANCE);
}

TEST(PointTests, EqualityOperatorOverloaded)
{
    Point a{1.0, 1.0};
    Point b{1.0, 1.0};
    CHECK(a == b);
}

TEST(PointTests, InequalityOperatorOverloaded)
{
    Point a{1.0, 1.0};
    Point b{2.0, 2.0};
    CHECK(a != b);
}
