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
Point test_point_a{1.0, 3.0}; /* arbitrary coordinates */
Point test_point_b{2.0, 4.0}; /* arbitrary coordinates */
Ray test_ray{test_point_a, test_point_b};

void initialize_test_variables(void)
{
    test_point_a = Point{1.0, 3.0}; /* arbitrary coordinates */
    test_point_b = Point{2.0, 4.0}; /* arbitrary coordinates */
    test_ray = Ray{test_point_a, test_point_b};
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
    CHECK(test_ray.back == test_point_a);
    CHECK(test_ray.front == test_point_b);
}

TEST(RayTests, TranslateModifiesPoints)
{
    double test_dx {-10.0};
    double test_dy {20.0};
    test_ray.translate(test_dx, test_dy);
    test_point_a.translate(test_dx, test_dy);
    test_point_b.translate(test_dx, test_dy);

    CHECK(test_ray.back == test_point_a);
    CHECK(test_ray.front == test_point_b);
}

TEST(RayTests, RotateModifiesPoints)
{
    Point center{0.0, 0.0};
    double angle{M_PI / 2}; /* 90 deg counter clockwise */
    test_ray.rotate(center, angle);
    test_point_a.rotate(center, angle);
    test_point_b.rotate(center, angle);

    CHECK(test_ray.back == test_point_a);
    CHECK(test_ray.front == test_point_b);
}

TEST(RayTests, EqualityOperatorOverloaded)
{
    Ray test_ray_2{test_point_a, test_point_b};

    CHECK(test_ray == test_ray_2);
}
