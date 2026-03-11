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
/* none */

/*============================================================================*/
/*                            Mock Implementations                            */
/*============================================================================*/


/*============================================================================*/
/*                                 Test Group                                 */
/*============================================================================*/
TEST_GROUP(RayTests)
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
TEST(RayTests, PointsMatchOnParameterizedConstructor)
{
    Point a{1.0, 3.0};
    Point b{2.0, 4.0};
    Ray ray{a, b};
    CHECK(ray.origin == a);
    CHECK(ray.direction == b);
}

TEST(RayTests, TranslateModifiesPoints)
{
    Point a{1.0, 3.0};
    Point b{2.0, 4.0};
    Ray ray{a, b};

    double user_dx {-10.0};
    double user_dy {20.0};
    ray.translate(user_dx, user_dy);
    a.translate(user_dx, user_dy);
    b.translate(user_dx, user_dy);

    CHECK(ray.origin == a);
    CHECK(ray.direction == b);
}

TEST(RayTests, RotateModifiesPoints)
{
    Point a{1.0, 3.0};
    Point b{2.0, 4.0};
    Ray ray{a, b};

    Point center{0.0, 0.0};
    double angle{M_PI / 2}; /* 90 deg counter clockwise */
    ray.rotate(center, angle);
    a.rotate(center, angle);
    b.rotate(center, angle);

    CHECK(ray.origin == a);
    CHECK(ray.direction == b);
}

TEST(RayTests, EqualityOperatorOverloaded)
{
    Point a{1.0, 3.0};
    Point b{2.0, 4.0};
    Ray ray_1{a, b};
    Ray ray_2{a, b};

    CHECK(ray_1 == ray_2);
}
