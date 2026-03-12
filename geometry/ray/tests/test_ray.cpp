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

/*============================================================================*/
/*                             Public Definitions                             */
/*============================================================================*/
geometry::Point test_origin {0.0, 0.0}; /* arbitrary coordinates */
double test_angle {M_PI / 4}; /* arbitrary angle */
geometry::Point test_direction {std::cos(test_angle), std::sin(test_angle)};
geometry::Ray test_ray {test_origin, test_angle};

void initialize_test_variables(void)
{
    test_origin = geometry::Point{0.0, 0.0}; /* arbitrary coordinates */
    test_direction = geometry::Point{std::cos(test_angle), std::sin(test_angle)};
    test_ray = geometry::Ray{test_origin, test_angle};
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

TEST(RayTests, TranslateModifiesPoints)
{
    double test_dx {-10.0};
    double test_dy {20.0};
    test_ray.translate(test_dx, test_dy);
    test_origin.translate(test_dx, test_dy);
    test_direction.translate(test_dx, test_dy);

    CHECK(test_ray.origin == test_origin);
    CHECK(test_ray.direction == test_direction);
}

TEST(RayTests, RotateModifiesPoints)
{
    geometry::Point center {0.0, 0.0};
    double angle {M_PI / 2}; /* 90 deg counter clockwise */
    test_ray.rotate(center, angle);
    test_origin.rotate(center, angle);
    test_direction.rotate(center, angle);

    CHECK(test_ray.origin == test_origin);
    CHECK(test_ray.direction == test_direction);
}

TEST(RayTests, EqualityOperatorOverloaded)
{
    geometry::Ray test_ray_2{test_origin, test_angle};

    CHECK(test_ray == test_ray_2);
}
