/*================================ FILE INFO =================================*/
/* Filename           : test_maze.cpp                                         */
/*                                                                            */
/* Test implementation for maze.cpp                                           */
/*                                                                            */
/*============================================================================*/

/*============================================================================*/
/*                               Include Files                                */
/*============================================================================*/
#include <cmath>
#include "point.hpp"
#include "rectangular_hitbox.hpp"
#include "maze.hpp"
#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>

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
TEST_GROUP(MazeTests)
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
TEST(MazeTests, PostWithSpecifiedAdjustmentsCreated)
{
    double adjustment {2.0};
    geometry::RectangularHitbox test_hitbox {
        maze::create_post(geometry::Point{0.0, 0.0}, adjustment, adjustment)
    };
    CHECK(test_hitbox.horizontal_size == (maze::OFFICIAL_POST_SIZE + adjustment));
    CHECK(test_hitbox.vertical_size == (maze::OFFICIAL_POST_SIZE + adjustment));
}

TEST(MazeTests, VerticalWallWithSpecifiedAdjustmentsCreated)
{
    double adjustment {2.0};
    geometry::RectangularHitbox test_hitbox {
        maze::create_vertical_wall(geometry::Point{0.0, 0.0}, adjustment, adjustment)
    };
    CHECK(test_hitbox.horizontal_size == (maze::OFFICIAL_WALL_WIDTH_SIZE + adjustment));
    CHECK(test_hitbox.vertical_size == (maze::OFFICIAL_WALL_LENGTH_SIZE + adjustment));
}

TEST(MazeTests, HorizontalWallWithSpecifiedAdjustmentsCreated)
{
    double adjustment {2.0};
    geometry::RectangularHitbox test_hitbox {
        maze::create_horizontal_wall(geometry::Point{0.0, 0.0}, adjustment, adjustment)
    };
    CHECK(test_hitbox.horizontal_size == (maze::OFFICIAL_WALL_LENGTH_SIZE + adjustment));
    CHECK(test_hitbox.vertical_size == (maze::OFFICIAL_WALL_WIDTH_SIZE + adjustment));
}
