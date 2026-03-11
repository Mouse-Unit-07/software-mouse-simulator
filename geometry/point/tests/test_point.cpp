/*================================ FILE INFO =================================*/
/* Filename           : test_point.cpp                                        */
/*                                                                            */
/* Test implementation for point.cpp                                          */
/*                                                                            */
/*============================================================================*/

/*============================================================================*/
/*                               Include Files                                */
/*============================================================================*/
#include "point.hpp"
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
    Point p;
    CHECK(p.x == 0);
    CHECK(p.y == 0);
}

TEST(PointTests, CoordinatesMatchOnParameterizedConstructor)
{
    Point p{1, 3};
    CHECK(p.x == 1);
    CHECK(p.y == 3);
}

TEST(PointTests, TranslateModifiesCoordinates)
{
    int user_x {1};
    int user_y {3};
    int user_dx {-10};
    int user_dy {20};
    Point p{user_x, user_y};
    p.translate(user_dx, user_dy);
    CHECK(p.x == user_x + user_dx);
    CHECK(p.y == user_y + user_dy);
}
