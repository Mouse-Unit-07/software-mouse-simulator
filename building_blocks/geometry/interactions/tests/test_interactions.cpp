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

using namespace geometry;

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
    Point test_point_a{0.0, 0.0};
    Ray test_ray{test_point_a, 0.0};
    RectangularHitbox test_hitbox{Point{5.0, 2.0}, 2.0, 4.0};

    for (int i{0}; i < 360; i++) {
        auto distance{compute_ray_hitbox_distance(test_ray, test_hitbox)};

        if (i <= 45) {
            CHECK(distance.has_value());
        } else {
            CHECK(!distance.has_value());
        }
        const double one_degree{M_PI / 180};
        test_ray.rotate(test_point_a, one_degree);
    }
}

TEST(InteractionsTests, RayAndHitboxDistanceComputable)
{
    Point test_point_a{0.0, 0.0};
    Ray test_ray{test_point_a, 0.0};
    RectangularHitbox test_hitbox{Point{5.0, 2.0}, 2.0, 4.0};

    auto distance{compute_ray_hitbox_distance(test_ray, test_hitbox)};
    CHECK(distance.has_value());
    
    double d{*distance};
    DOUBLES_EQUAL(4.0, d, 1e-6);
}

TEST(InteractionsTests, RayAndHitboxDistanceComputableAfterTranslation)
{
    Point test_point_a{0.0, 0.0};
    Ray test_ray{test_point_a, 0.0};
    RectangularHitbox test_hitbox{Point{5.0, 2.0}, 2.0, 4.0};

    test_ray.translate(1.0, 2.0);
    auto distance{compute_ray_hitbox_distance(test_ray, test_hitbox)};
    
    
    double d{*distance};
    DOUBLES_EQUAL(3.0, d, 1e-6);
}

TEST(InteractionsTests, RayAndHitboxDistanceComputableAfterRotation)
{
    Point test_point_a{1.0, 0.0};
    Ray test_ray{test_point_a, 0.0};
    RectangularHitbox test_hitbox{Point{-5.0, 2.0}, 2.0, 4.0};

    test_ray.rotate(test_point_a, M_PI);
    auto distance{compute_ray_hitbox_distance(test_ray, test_hitbox)};
    
    double d{*distance};
    DOUBLES_EQUAL(5.0, d, 1e-6);
}

TEST(InteractionsTests, HitboxCollisionDetectable)
{
    RectangularHitbox test_hitbox_1{Point{0.0, 0.0}, 1.0, 1.0};
    RectangularHitbox test_hitbox_2{Point{0.0, 0.0}, 1.0, 1.0};
    RectangularHitbox test_hitbox_3{Point{0.0, 0.0}, 1.0, 1.0};

    test_hitbox_2.translate(1.0, 1.0);
    test_hitbox_3.rotate(test_hitbox_3.center, M_PI / 4);
    test_hitbox_3.translate(0.0, -1.0);

    CHECK(do_hitboxes_overlap(test_hitbox_1, test_hitbox_2));
    CHECK(do_hitboxes_overlap(test_hitbox_1, test_hitbox_3));
    CHECK(!do_hitboxes_overlap(test_hitbox_2, test_hitbox_3));
}
