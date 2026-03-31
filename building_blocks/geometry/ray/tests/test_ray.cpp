/*================================ FILE INFO =================================*/
/* Filename           : test_ray.cpp                                          */
/*                                                                            */
/* Test implementation for ray.cpp                                            */
/*                                                                            */
/*============================================================================*/

/*============================================================================*/
/*                               Include Files                                */
/*============================================================================*/
#include <cmath>
#include "point.hpp"
#include "ray.hpp"
#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>

using namespace geometry;

/*============================================================================*/
/*                             Public Definitions                             */
/*============================================================================*/
Point test_origin{0.0, 0.0}; /* arbitrary coordinates */
double test_angle{M_PI / 4}; /* arbitrary angle */
Point test_direction{std::cos(test_angle), std::sin(test_angle)};
Ray test_ray{test_origin, test_angle};

void initialize_test_variables(void)
{
    test_origin = Point{0.0, 0.0}; /* arbitrary coordinates */
    test_direction = Point{std::cos(test_angle), std::sin(test_angle)};
    test_ray = Ray{test_origin, test_angle};
}

/*============================================================================*/
/*                            Mock Implementations                            */
/*============================================================================*/
/* none */

/*============================================================================*/
/*                                 Test Group                                 */
/*============================================================================*/
TEST_GROUP(RayTests)
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
TEST(RayTests, PointsMatchOnParameterizedConstructor)
{
    CHECK(test_ray.origin == test_origin);
    CHECK(test_ray.direction == test_direction);
}

TEST(RayTests, TranslateModifiesOrigin)
{
    double test_dx{-10.0};
    double test_dy{20.0};
    test_ray.translate(test_dx, test_dy);
    test_origin.translate(test_dx, test_dy);
    
    CHECK(test_ray.origin == test_origin);
}

TEST(RayTests, RotateModifiesOrigin)
{
    Point center{5.0, 6.0};
    double angle{M_PI / 2}; /* 90 deg counter clockwise */
    test_ray.rotate(center, angle);
    test_origin.rotate(center, angle);

    CHECK(test_ray.origin == test_origin);
}


TEST(RayTests, TranslateDoesNotModifyDirection)
{
    double test_dx{-10.0};
    double test_dy{20.0};
    test_ray.translate(test_dx, test_dy);

    CHECK(test_ray.direction == test_direction);
}

TEST(RayTests, RotateModifiesDirectionAboutOrigin)
{
    Point center{5.0, 6.0};
    double angle{M_PI / 2}; /* 90 deg counter clockwise */
    test_ray.rotate(center, angle);
    test_direction.rotate(Point{0.0, 0.0}, angle);

    CHECK(test_ray.direction == test_direction);
}


TEST(RayTests, EqualityOperatorOverloaded)
{
    Ray test_ray_2{test_origin, test_angle};

    CHECK(test_ray == test_ray_2);
}

TEST(RayTests, InequalityOperatorOverloaded)
{
    Ray test_ray_2{Point{1.0, 1.0}, test_angle};

    CHECK(test_ray != test_ray_2);
}
