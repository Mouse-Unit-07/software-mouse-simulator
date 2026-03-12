/*================================ FILE INFO =================================*/
/* Filename           : test_interactions.cpp                                 */
/*                                                                            */
/* Test implementation for interactions.cpp                                   */
/*                                                                            */
/*============================================================================*/

/*============================================================================*/
/*                               Include Files                                */
/*============================================================================*/
#include <cmath>
#include <optional>
#include "point.hpp"
#include "rectangular_hitbox.hpp"
#include "ray.hpp"
#include "interactions.hpp"
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
TEST_GROUP(InteractionsTests)
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
TEST(InteractionsTests, RayAndHitboxIntersectionDetectable)
{
    geometry::Point test_point_a {0.0, 0.0};
    geometry::Point test_point_b {2.0, 0.0};
    geometry::Ray test_ray {test_point_a, test_point_b};
    geometry::RectangularHitbox test_hitbox{geometry::Point{5.0, 2.0}, 2.0, 4.0};

    for (int i = 0; i < 360; i++) {
        auto distance = geometry::ray_hitbox_distance(test_ray, test_hitbox);

        if (i <= 45) {
            CHECK(distance.has_value());
        } else {
            CHECK(!distance.has_value());
        }
        const double one_degree = M_PI / 180;
        test_ray.rotate(test_point_a, one_degree);
    }
}

TEST(InteractionsTests, RayAndHitboxDistanceComputable)
{
    geometry::Point test_point_a {0.0, 0.0};
    geometry::Point test_point_b {2.0, 0.0};
    geometry::Ray test_ray {test_point_a, test_point_b};
    geometry::RectangularHitbox test_hitbox{geometry::Point{5.0, 2.0}, 2.0, 4.0};

    auto distance = geometry::ray_hitbox_distance(test_ray, test_hitbox);
    CHECK(distance.has_value());
    
    double d = *distance;
    DOUBLES_EQUAL(2.0, d, 1e-6);
}
