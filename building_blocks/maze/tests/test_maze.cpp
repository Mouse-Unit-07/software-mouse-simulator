/*================================ FILE INFO =================================*/
/* Filename           : test_maze.cpp                                         */
/*                                                                            */
/* Test implementation for maze.cpp                                           */
/*                                                                            */
/*============================================================================*/

/*============================================================================*/
/*                               Include Files                                */
/*============================================================================*/
#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include "point.hpp"
#include "ray.hpp"
#include "rectangular_hitbox.hpp"
#include "maze.hpp"
#include "mouse.hpp"

#include <iostream>

using namespace maze;

/*============================================================================*/
/*                             Public Definitions                             */
/*============================================================================*/
constexpr double FLOAT_TOLERANCE{1e-6};

constexpr double OFFICIAL_POST_SIZE{12.07};
constexpr double OFFICIAL_WALL_LENGTH_SIZE{166.37};
constexpr double OFFICIAL_WALL_WIDTH_SIZE{12.07};

constexpr double CELL_SIZE{OFFICIAL_WALL_LENGTH_SIZE + OFFICIAL_POST_SIZE};

/* count all unique touches, including hitbox corners */
int count_touching_obstacles(const Maze &maze)
{
    const auto &obstacles{maze.obstacles};
    int touching_count{0};

    for (size_t i{0}; i < obstacles.size(); i++) {
        for (size_t j{i + 1}; j < obstacles.size(); j++) {
            const auto &a{obstacles.at(i)};
            const auto &b{obstacles.at(j)};

            double dx{fabs(a.center.x - b.center.x)};
            double dy{fabs(a.center.y - b.center.y)};

            double allowed_x{(a.horizontal_size / 2.0) + (b.horizontal_size / 2.0)};
            double allowed_y{(a.vertical_size / 2.0) + (b.vertical_size / 2.0)};

            bool horizontal_touch{(fabs(dx - allowed_x) < FLOAT_TOLERANCE)
                                  && (dy < (allowed_y + FLOAT_TOLERANCE))};

            bool vertical_touch{(fabs(dy - allowed_y) < FLOAT_TOLERANCE)
                                && (dx < (allowed_x + FLOAT_TOLERANCE))};

            if (horizontal_touch || vertical_touch) {
                touching_count++;
            }
        }
    }

    return touching_count;
}

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
TEST(MazeTests, ThrowsOnEmptyAsciiMaze)
{
    std::vector<std::string> ascii{};

    try {
        build_maze_from_ascii(ascii, 1.0, 1.0);
        FAIL("Expected exception not thrown");
    } catch (const std::invalid_argument &e) {
        STRCMP_EQUAL("ASCII maze is empty", e.what());
    }
}

TEST(MazeTests, ThrowsOnJaggedAsciiMaze)
{
    std::vector<std::string> ascii
    {
        "+-+",
        "|S|",
        "+-+-+"
    };

    try {
        build_maze_from_ascii(ascii, 1.0, 1.0);
        FAIL("Expected exception not thrown");
    } catch (const std::invalid_argument &e) {
        STRCMP_CONTAINS("ASCII maze is jagged", e.what());
    }
}

TEST(MazeTests, ThrowsOnInvalidCharacterInAsciiMaze)
{
    std::vector<std::string> ascii
    {
        "+-+",
        "|X|",
        "+-+"
    };

    try {
        build_maze_from_ascii(ascii, 1.0, 1.0);
        FAIL("Expected exception not thrown");
    } catch (const std::invalid_argument &e) {
        STRCMP_CONTAINS("Invalid character 'X'", e.what());
    }
}

TEST(MazeTests, ThrowsWhenNoStartPosition)
{
    std::vector<std::string> ascii
    {
        "+-+",
        "| |",
        "+-+"
    };

    try {
        build_maze_from_ascii(ascii, 1.0, 1.0);
        FAIL("Expected exception not thrown");
    } catch (const std::invalid_argument &e) {
        STRCMP_EQUAL("No 'S' start position found", e.what());
    }
}

TEST(MazeTests, ThrowsWhenMultipleStartPositions)
{
    std::vector<std::string> ascii
    {
        "+-+",
        "|S|",
        "+S+"
    };

    try {
        build_maze_from_ascii(ascii, 1.0, 1.0);
        FAIL("Expected exception not thrown");
    } catch (const std::invalid_argument &e) {
        STRCMP_EQUAL("Multiple 'S' start positions found", e.what());
    }
}

TEST(MazeTests, CorrectNumberOfMazeDimensions)
{
    std::vector<std::string> ascii
    {
        "+-+-+",
        "|S  |",
        "+ +-+",
        "|   |",
        "+-+-+"
    };
    Maze maze{build_maze_from_ascii(ascii, 1.0, 1.0)};

    CHECK_EQUAL(2, maze.rows);
    CHECK_EQUAL(2, maze.cols);
}

TEST(MazeTests, CellCoordinatesAreTopLeftOrigin)
{
    std::vector<std::string> ascii
    {
        "+-+ +",
        "|S|  ",
        "+-+-+",
        "  |  ",
        "+ +-+"
    };
    Maze maze{build_maze_from_ascii(ascii, 1.0, 1.0)};
    const Cell &top_left{maze.get_cell(0, 0)};
    const Cell &bottom_right{maze.get_cell(1, 1)};

    LONGS_EQUAL(8, top_left.obstacles.size());
    LONGS_EQUAL(7, bottom_right.obstacles.size());
}

TEST(MazeTests, MouseStartCoordinatesCreated)
{
    std::vector<std::string> ascii
    {
        "+-+",
        "|S|",
        "+-+"
    };
    Maze maze{build_maze_from_ascii(ascii, 1.0, 1.0)};

    CHECK(maze.mouse_start.x > 0);
    CHECK(maze.mouse_start.y > 0);
}

TEST(MazeTests, MouseStartPlacedAtCellCenter)
{
    std::vector<std::string> ascii
    {
        "+-+-+",
        "|S  |",
        "+ +-+",
        "|   |",
        "+-+-+"
    };
    Maze maze{build_maze_from_ascii(ascii, 1.0, 1.0)};
    double expected_x{CELL_SIZE / 2.0};
    double expected_y{CELL_SIZE / 2.0};

    DOUBLES_EQUAL(expected_x, maze.mouse_start.x, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(expected_y, maze.mouse_start.y, FLOAT_TOLERANCE);
}

TEST(MazeTests, MouseStartPlacedCorrectlyOnSizeAdjustedMaze)
{
    std::vector<std::string> ascii
    {
        "+-+-+",
        "|   |",
        "+ +-+",
        "|  S|",
        "+-+-+"
    };
    Maze maze{build_maze_from_ascii(ascii, 1.5, 0.5)};
    double expected_x{(maze.cell_size) + (maze.cell_size / 2.0)};
    double expected_y{(maze.cell_size) + (maze.cell_size / 2.0)};

    DOUBLES_EQUAL(expected_x, maze.mouse_start.x, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(expected_y, maze.mouse_start.y, FLOAT_TOLERANCE);
}

TEST(MazeTests, CorrectNumberOfPostCreated)
{
    std::vector<std::string> ascii
    {
        "+ +",
        " S ",
        "+ +"
    };
    Maze maze{build_maze_from_ascii(ascii, 1.0, 1.0)};
    int post_count{0};

    for (const auto &obstacle : maze.obstacles) {
        double width{obstacle.horizontal_size};
        double height{obstacle.vertical_size};

        if (fabs(width - OFFICIAL_POST_SIZE) < FLOAT_TOLERANCE
            && fabs(height - OFFICIAL_POST_SIZE) < FLOAT_TOLERANCE) {
            post_count++;
        }
    }

    LONGS_EQUAL(4, post_count);
}

TEST(MazeTests, CorrectNumberOfVerticalWalls)
{
    std::vector<std::string> ascii
    {
        "+ +",
        "|S|",
        "+ +"
    };
    Maze maze{build_maze_from_ascii(ascii, 1.0, 1.0)};
    int wall_count{0};

    for (const auto &obstacle : maze.obstacles) {
        if (obstacle.vertical_size > obstacle.horizontal_size) {
            wall_count++;
        }
    }

    LONGS_EQUAL(2, wall_count);
}

TEST(MazeTests, CorrectNumberOfHorizontalWalls)
{
    std::vector<std::string> ascii
    {
        "+-+",
        " S ",
        "+-+"
    };
    Maze maze{build_maze_from_ascii(ascii, 1.0, 1.0)};
    int wall_count{0};

    for (const auto &obstacle : maze.obstacles) {
        if (obstacle.vertical_size < obstacle.horizontal_size) {
            wall_count++;
        }
    }

    LONGS_EQUAL(2, wall_count);
}

TEST(MazeTests, VerticalWallSharedBetweenCells)
{
    std::vector<std::string> ascii
    {
        "+-+-+",
        "|S| |",
        "+-+-+"
    };
    Maze maze{build_maze_from_ascii(ascii, 1.0, 1.0)};
    const Cell &left{maze.get_cell(0, 0)};
    const Cell &right{maze.get_cell(0, 1)};
    bool shared{false};

    for (size_t a : left.obstacles) {
        for (size_t b : right.obstacles) {
            if (a == b) {
                shared = true;
            }
        }
    }

    CHECK(shared);
}

TEST(MazeTests, HorizontalWallSharedBetweenCells)
{
    std::vector<std::string> ascii
    {
        "+-+",
        "|S|",
        "+-+",
        "| |",
        "+-+"
    };
    Maze maze{build_maze_from_ascii(ascii, 1.0, 1.0)};
    const Cell &bottom{maze.get_cell(0, 0)};
    const Cell &top{maze.get_cell(1, 0)};
    bool shared{false};

    for (size_t a : bottom.obstacles) {
        for (size_t b : top.obstacles) {
            if (a == b) {
                shared = true;
            }
        }
    }

    CHECK(shared);
}

TEST(MazeTests, SingleCellWallsTouchPosts)
{
    std::vector<std::string> ascii
    {
        "+-+",
        "|S|",
        "+-+"
    };
    Maze maze{build_maze_from_ascii(ascii, 1.0, 1.0)};
    int touching_count{count_touching_obstacles(maze)};

    /* 8 full unique touches + 4 diagonal corner touches */
    LONGS_EQUAL(12, touching_count);
}

TEST(MazeTests, PostAndWallSizeAdjustmentsModifyMazeSizeFields)
{
    const double post_size_adjustment{1.1};
    const double wall_size_adjustment{1.2};

    std::vector<std::string> ascii
    {
        "+-+",
        "|S|",
        "+-+"
    };
    Maze maze{build_maze_from_ascii(ascii, post_size_adjustment, wall_size_adjustment)};

    DOUBLES_EQUAL(maze.post_size, OFFICIAL_POST_SIZE * post_size_adjustment, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(maze.wall_length_size, OFFICIAL_WALL_LENGTH_SIZE * wall_size_adjustment,
                  FLOAT_TOLERANCE);
}

TEST(MazeTests, PostAndWallSizeAdjustmentsModifyMazeObjectSizes)
{
    const double post_size_adjustment{1.1};
    const double wall_size_adjustment{1.2};

    std::vector<std::string> ascii
    {
        "+-+",
        "|S|",
        "+-+"
    };
    Maze maze{build_maze_from_ascii(ascii, post_size_adjustment, wall_size_adjustment)};

    for (const auto &o : maze.obstacles) {
        if (o.horizontal_size > o.vertical_size) {
            DOUBLES_EQUAL(OFFICIAL_WALL_LENGTH_SIZE * wall_size_adjustment, o.horizontal_size,
                          FLOAT_TOLERANCE);
            DOUBLES_EQUAL(OFFICIAL_WALL_WIDTH_SIZE * post_size_adjustment, o.vertical_size,
                          FLOAT_TOLERANCE);
        }

        if (o.vertical_size > o.horizontal_size) {
            DOUBLES_EQUAL(OFFICIAL_WALL_WIDTH_SIZE * post_size_adjustment, o.horizontal_size,
                          FLOAT_TOLERANCE);
            DOUBLES_EQUAL(OFFICIAL_WALL_LENGTH_SIZE * wall_size_adjustment, o.vertical_size,
                          FLOAT_TOLERANCE);
        }

        if (fabs(o.horizontal_size - o.vertical_size) < FLOAT_TOLERANCE) {
            DOUBLES_EQUAL(OFFICIAL_POST_SIZE * post_size_adjustment, o.horizontal_size,
                          FLOAT_TOLERANCE);
            DOUBLES_EQUAL(OFFICIAL_POST_SIZE * post_size_adjustment, o.vertical_size,
                          FLOAT_TOLERANCE);
        }
    }
}

TEST(MazeTests, PostSizeAdjustedWallsAndPostsTouch)
{
    const double post_size_adjustment{0.9};
    const double wall_size_adjustment{0.8};

    std::vector<std::string> ascii
    {
        "+-+",
        "|S|",
        "+-+"
    };
    Maze maze{build_maze_from_ascii(ascii, post_size_adjustment, wall_size_adjustment)};
    int touching_count{count_touching_obstacles(maze)};

    /* 8 full unique touches + 4 diagonal corner touches */
    LONGS_EQUAL(12, touching_count);
}

TEST(MazeTests, ClosedSpaceRayAlwaysReturnsDistance)
{
    std::vector<std::string> ascii
    {
        "+-+",
        "|S|",
        "+-+"
    };
    Maze maze{build_maze_from_ascii(ascii, 1.0, 1.0)};

    mouse::Mouse mouse;
    mouse.translate(maze.mouse_start.x, maze.mouse_start.y);

    for (int i{0}; i < 8; i++) {
        double distance_1{
                compute_ray_distance_in_closed_space(maze, mouse.hitbox.center, mouse.ir_1_sensor)};
        double distance_2{
                compute_ray_distance_in_closed_space(maze, mouse.hitbox.center, mouse.ir_2_sensor)};
        double distance_3{
                compute_ray_distance_in_closed_space(maze, mouse.hitbox.center, mouse.ir_3_sensor)};
        double distance_4{
                compute_ray_distance_in_closed_space(maze, mouse.hitbox.center, mouse.ir_4_sensor)};

        CHECK(distance_1 > 0.0);
        CHECK(distance_2 > 0.0);
        CHECK(distance_3 > 0.0);
        CHECK(distance_4 > 0.0);

        mouse.rotate(M_PI / 4);
    }
}

TEST(MazeTests, IdealRayDistancesAreKnown)
{
    std::vector<std::string> ascii
    {
        "+-+",
        "|S|",
        "+-+"
    };
    Maze maze{build_maze_from_ascii(ascii, 1.0, 1.0)};

    mouse::Mouse mouse;
    mouse.translate(maze.mouse_start.x, maze.mouse_start.y);

    for (int i{0}; i < 4; i++) {
        double distance_1{
                compute_ray_distance_in_closed_space(maze, mouse.hitbox.center, mouse.ir_1_sensor)};
        double distance_2{
                compute_ray_distance_in_closed_space(maze, mouse.hitbox.center, mouse.ir_2_sensor)};
        double distance_3{
                compute_ray_distance_in_closed_space(maze, mouse.hitbox.center, mouse.ir_3_sensor)};
        double distance_4{
                compute_ray_distance_in_closed_space(maze, mouse.hitbox.center, mouse.ir_4_sensor)};

        DOUBLES_EQUAL(57.945, distance_1, 1e-3);
        DOUBLES_EQUAL(62.82644, distance_2, 1e-3);
        DOUBLES_EQUAL(62.82644, distance_3, 1e-3);
        DOUBLES_EQUAL(57.945, distance_4, 1e-3);

        mouse.rotate(M_PI / 2);
    }
}

TEST(MazeTests, RaysReturnPositiveDistancesInClosedThreeByThree)
{
    std::vector<std::string> ascii
    {
        "+-+-+-+",
        "|     |",
        "+     +",
        "|  S  |",
        "+     +",
        "|     |",
        "+-+-+-+",
    };
    Maze maze{build_maze_from_ascii(ascii, 1.0, 1.0)};

    mouse::Mouse mouse;
    mouse.translate(maze.mouse_start.x, maze.mouse_start.y);

    for (int i{0}; i < 8; i++) {
        double distance_1{
                compute_ray_distance_in_closed_space(maze, mouse.hitbox.center, mouse.ir_1_sensor)};
        double distance_2{
                compute_ray_distance_in_closed_space(maze, mouse.hitbox.center, mouse.ir_2_sensor)};
        double distance_3{
                compute_ray_distance_in_closed_space(maze, mouse.hitbox.center, mouse.ir_3_sensor)};
        double distance_4{
                compute_ray_distance_in_closed_space(maze, mouse.hitbox.center, mouse.ir_4_sensor)};

        CHECK(distance_1 > 0.0);
        CHECK(distance_2 > 0.0);
        CHECK(distance_3 > 0.0);
        CHECK(distance_4 > 0.0);

        mouse.rotate(M_PI / 4);
    }
}

TEST(MazeTests, OpenSpaceReturnsZeroWhenNoHit)
{
    std::vector<std::string> ascii
    {
        "       ",
        "       ",
        "       ",
        "   S   ",
        "       ",
        "       ",
        "       ",
    };
    Maze maze{build_maze_from_ascii(ascii, 1.0, 1.0)};

    mouse::Mouse mouse;
    mouse.translate(maze.mouse_start.x, maze.mouse_start.y);

    for (int i{0}; i < 8; i++) {
        double distance_1{
                compute_ray_distance_in_open_space(maze, mouse.hitbox.center, mouse.ir_1_sensor)};
        double distance_2{
                compute_ray_distance_in_open_space(maze, mouse.hitbox.center, mouse.ir_2_sensor)};
        double distance_3{
                compute_ray_distance_in_open_space(maze, mouse.hitbox.center, mouse.ir_3_sensor)};
        double distance_4{
                compute_ray_distance_in_open_space(maze, mouse.hitbox.center, mouse.ir_4_sensor)};

        DOUBLES_EQUAL(0.0, distance_1, FLOAT_TOLERANCE);
        DOUBLES_EQUAL(0.0, distance_2, FLOAT_TOLERANCE);
        DOUBLES_EQUAL(0.0, distance_3, FLOAT_TOLERANCE);
        DOUBLES_EQUAL(0.0, distance_4, FLOAT_TOLERANCE);

        mouse.rotate(M_PI / 4);
    }
}

TEST(MazeTests, ClosedSpaceThrowsWhenNoHit)
{
    std::vector<std::string> ascii
    {
        "       ",
        "       ",
        "       ",
        "   S   ",
        "       ",
        "       ",
        "       ",
    };
    Maze maze{build_maze_from_ascii(ascii, 1.0, 1.0)};

    mouse::Mouse mouse;
    mouse.translate(maze.mouse_start.x, maze.mouse_start.y);

    CHECK_THROWS(std::runtime_error, compute_ray_distance_in_closed_space(maze, mouse.hitbox.center,
                                                                          mouse.ir_1_sensor));
    CHECK_THROWS(std::runtime_error, compute_ray_distance_in_closed_space(maze, mouse.hitbox.center,
                                                                          mouse.ir_2_sensor));
    CHECK_THROWS(std::runtime_error, compute_ray_distance_in_closed_space(maze, mouse.hitbox.center,
                                                                          mouse.ir_3_sensor));
    CHECK_THROWS(std::runtime_error, compute_ray_distance_in_closed_space(maze, mouse.hitbox.center,
                                                                          mouse.ir_4_sensor));
}

TEST(MazeTests, RayReturnsNearestObstacleDistance)
{
    std::vector<std::string> ascii
    {
        "+-+-+-+",
        "|     |",
        "+ +-+ +",
        "| |S| |",
        "+ +-+ +",
        "|     |",
        "+-+-+-+",
    };
    Maze maze{build_maze_from_ascii(ascii, 1.0, 1.0)};

    mouse::Mouse mouse;
    mouse.translate(maze.mouse_start.x, maze.mouse_start.y);

    for (int i{0}; i < 8; i++) {
        double distance_1{
                compute_ray_distance_in_closed_space(maze, mouse.hitbox.center, mouse.ir_1_sensor)};
        double distance_2{
                compute_ray_distance_in_closed_space(maze, mouse.hitbox.center, mouse.ir_2_sensor)};
        double distance_3{
                compute_ray_distance_in_closed_space(maze, mouse.hitbox.center, mouse.ir_3_sensor)};
        double distance_4{
                compute_ray_distance_in_closed_space(maze, mouse.hitbox.center, mouse.ir_4_sensor)};

        CHECK(distance_1 < maze.cell_size);
        CHECK(distance_2 < maze.cell_size);
        CHECK(distance_3 < maze.cell_size);
        CHECK(distance_4 < maze.cell_size);

        mouse.rotate(M_PI / 4);
    }
}

TEST(MazeTests, HitboxAtCenterDoesNotCollide)
{
    std::vector<std::string> ascii
    {
        "+-+",
        "|S|",
        "+-+"
    };
    Maze maze{build_maze_from_ascii(ascii, 1.0, 1.0)};

    mouse::Mouse mouse;
    mouse.translate(maze.mouse_start.x, maze.mouse_start.y);

    CHECK(!does_hitbox_collide_with_maze(maze, mouse.hitbox));
}

TEST(MazeTests, HitboxCollidesWhenMovedIntoWall)
{
    std::vector<std::string> ascii
    {
        "+-+",
        "|S|",
        "+-+"
    };
    Maze maze{build_maze_from_ascii(ascii, 1.0, 1.0)};

    mouse::Mouse mouse;
    mouse.translate(maze.mouse_start.x, maze.mouse_start.y);
    mouse.translate(maze.cell_size / 2, 0.0);

    CHECK(does_hitbox_collide_with_maze(maze, mouse.hitbox));
}

TEST(MazeTests, HitboxOutsideMazeIsCollision)
{
    std::vector<std::string> ascii
    {
        "+-+",
        "|S|",
        "+-+"
    };
    Maze maze{build_maze_from_ascii(ascii, 1.0, 1.0)};

    mouse::Mouse mouse;
    mouse.translate(maze.cell_size * 5, maze.cell_size * 5);

    CHECK(does_hitbox_collide_with_maze(maze, mouse.hitbox));
}

TEST(MazeTests, MouseMovingNearWallsNoCollision)
{
    std::vector<std::string> ascii
    {
        "+-+",
        "|S|",
        "+-+"
    };
    Maze maze{build_maze_from_ascii(ascii, 1.0, 1.0)};

    mouse::Mouse mouse;
    mouse.translate(maze.mouse_start.x, maze.mouse_start.y);

    double distance_to_wall_center{(OFFICIAL_POST_SIZE + OFFICIAL_WALL_LENGTH_SIZE) / 2};
    double horizontal_distance_to_cell_wall{(distance_to_wall_center
                                             - (mouse.hitbox.horizontal_size / 2)
                                             - (OFFICIAL_POST_SIZE / 2))
                                            - 1};
    double vertical_distance_to_cell_wall{
            (distance_to_wall_center - (mouse.hitbox.vertical_size / 2) - (OFFICIAL_POST_SIZE / 2))
            - 1};

    mouse.translate(0.0, vertical_distance_to_cell_wall);
    CHECK_FALSE(does_hitbox_collide_with_maze(maze, mouse.hitbox));
    mouse.translate(0.0, -vertical_distance_to_cell_wall);

    mouse.translate(-horizontal_distance_to_cell_wall, 0.0);
    CHECK_FALSE(does_hitbox_collide_with_maze(maze, mouse.hitbox));
    mouse.translate(horizontal_distance_to_cell_wall, 0.0);

    mouse.translate(-0.0, -vertical_distance_to_cell_wall);
    CHECK_FALSE(does_hitbox_collide_with_maze(maze, mouse.hitbox));
    mouse.translate(0.0, vertical_distance_to_cell_wall);

    mouse.translate(horizontal_distance_to_cell_wall, 0.0);
    CHECK_FALSE(does_hitbox_collide_with_maze(maze, mouse.hitbox));
    mouse.translate(-horizontal_distance_to_cell_wall, 0.0);
}

TEST(MazeTests, MouseMovingJustOntoWallsCausesCollision)
{
    std::vector<std::string> ascii
    {
        "+-+",
        "|S|",
        "+-+"
    };
    Maze maze{build_maze_from_ascii(ascii, 1.0, 1.0)};

    mouse::Mouse mouse;
    mouse.translate(maze.mouse_start.x, maze.mouse_start.y);

    double distance_to_wall_center{(OFFICIAL_POST_SIZE + OFFICIAL_WALL_LENGTH_SIZE) / 2};
    double horizontal_distance_to_cell_wall{(distance_to_wall_center
                                             - (mouse.hitbox.horizontal_size / 2)
                                             - (OFFICIAL_POST_SIZE / 2))};
    double vertical_distance_to_cell_wall{(
            distance_to_wall_center - (mouse.hitbox.vertical_size / 2) - (OFFICIAL_POST_SIZE / 2))};

    mouse.translate(0.0, vertical_distance_to_cell_wall);
    CHECK(does_hitbox_collide_with_maze(maze, mouse.hitbox));
    mouse.translate(0.0, -vertical_distance_to_cell_wall);

    mouse.translate(-horizontal_distance_to_cell_wall, 0.0);
    CHECK(does_hitbox_collide_with_maze(maze, mouse.hitbox));
    mouse.translate(horizontal_distance_to_cell_wall, 0.0);

    mouse.translate(-0.0, -vertical_distance_to_cell_wall);
    CHECK(does_hitbox_collide_with_maze(maze, mouse.hitbox));
    mouse.translate(0.0, vertical_distance_to_cell_wall);

    mouse.translate(horizontal_distance_to_cell_wall, 0.0);
    CHECK(does_hitbox_collide_with_maze(maze, mouse.hitbox));
    mouse.translate(-horizontal_distance_to_cell_wall, 0.0);
}

TEST(MazeTests, MouseCollidesWithNearWalls)
{
    std::vector<std::string> ascii
    {
        " - ",
        "|S|",
        " - "
    };
    Maze maze{build_maze_from_ascii(ascii, 1.0, 1.0)};

    mouse::Mouse mouse;
    mouse.translate(maze.mouse_start.x, maze.mouse_start.y);

    double distance_to_near_wall{maze.cell_size / 2};

    mouse.translate(-(distance_to_near_wall), 0.0);
    CHECK(does_hitbox_collide_with_maze(maze, mouse.hitbox));
    mouse.translate(distance_to_near_wall, 0.0);

    mouse.translate(distance_to_near_wall, 0.0);
    CHECK(does_hitbox_collide_with_maze(maze, mouse.hitbox));
    mouse.translate(-(distance_to_near_wall), 0.0);

    mouse.translate(0.0, -(distance_to_near_wall));
    CHECK(does_hitbox_collide_with_maze(maze, mouse.hitbox));
    mouse.translate(0.0, distance_to_near_wall);

    mouse.translate(0.0, distance_to_near_wall);
    CHECK(does_hitbox_collide_with_maze(maze, mouse.hitbox));
    mouse.translate(0.0, -(distance_to_near_wall));
}

TEST(MazeTests, MouseCollidesWithNearPosts)
{
    std::vector<std::string> ascii
    {
        "+ +",
        " S ",
        "+ +"
    };
    Maze maze{build_maze_from_ascii(ascii, 1.0, 1.0)};

    mouse::Mouse mouse;
    mouse.translate(maze.mouse_start.x, maze.mouse_start.y);

    double distance_to_near_wall{maze.cell_size / 2};

    mouse.translate(distance_to_near_wall, distance_to_near_wall);
    CHECK(does_hitbox_collide_with_maze(maze, mouse.hitbox));
    mouse.translate(-(distance_to_near_wall), -(distance_to_near_wall));

    mouse.translate(-(distance_to_near_wall), distance_to_near_wall);
    CHECK(does_hitbox_collide_with_maze(maze, mouse.hitbox));
    mouse.translate(distance_to_near_wall, -(distance_to_near_wall));

    mouse.translate(-(distance_to_near_wall), -(distance_to_near_wall));
    CHECK(does_hitbox_collide_with_maze(maze, mouse.hitbox));
    mouse.translate(distance_to_near_wall, distance_to_near_wall);

    mouse.translate(distance_to_near_wall, -(distance_to_near_wall));
    CHECK(does_hitbox_collide_with_maze(maze, mouse.hitbox));
    mouse.translate(-(distance_to_near_wall), distance_to_near_wall);
}

TEST(MazeTests, MouseCollidesWithFarWalls)
{
    std::vector<std::string> ascii
    {
        " - - - ",
        "|     |",
        "       ",
        "|  S  |",
        "       ",
        "|     |",
        " - - - ",
    };
    Maze maze{build_maze_from_ascii(ascii, 1.0, 1.0)};

    mouse::Mouse mouse;
    mouse.translate(maze.mouse_start.x, maze.mouse_start.y);

    double distance_to_far_wall{maze.cell_size + (maze.cell_size / 2)};

    mouse.translate(-(distance_to_far_wall), 0.0);
    CHECK(does_hitbox_collide_with_maze(maze, mouse.hitbox));
    mouse.translate(distance_to_far_wall, 0.0);

    mouse.translate(distance_to_far_wall, 0.0);
    CHECK(does_hitbox_collide_with_maze(maze, mouse.hitbox));
    mouse.translate(-(distance_to_far_wall), 0.0);

    mouse.translate(0.0, -(distance_to_far_wall));
    CHECK(does_hitbox_collide_with_maze(maze, mouse.hitbox));
    mouse.translate(0.0, distance_to_far_wall);

    mouse.translate(0.0, distance_to_far_wall);
    CHECK(does_hitbox_collide_with_maze(maze, mouse.hitbox));
    mouse.translate(0.0, -(distance_to_far_wall));
}

TEST(MazeTests, MouseCollidesWithFarPosts)
{
    std::vector<std::string> ascii
    {
        "+ + + +",
        "       ",
        "+     +",
        "   S   ",
        "+     +",
        "       ",
        "+ + + +",
    };
    Maze maze{build_maze_from_ascii(ascii, 1.0, 1.0)};

    mouse::Mouse mouse;
    mouse.translate(maze.mouse_start.x, maze.mouse_start.y);

    double distance_to_far_wall{maze.cell_size + (maze.cell_size / 2)};

    mouse.translate(distance_to_far_wall, distance_to_far_wall);
    CHECK(does_hitbox_collide_with_maze(maze, mouse.hitbox));
    mouse.translate(-(distance_to_far_wall), -(distance_to_far_wall));

    mouse.translate(-(distance_to_far_wall), distance_to_far_wall);
    CHECK(does_hitbox_collide_with_maze(maze, mouse.hitbox));
    mouse.translate(distance_to_far_wall, -(distance_to_far_wall));

    mouse.translate(-(distance_to_far_wall), -(distance_to_far_wall));
    CHECK(does_hitbox_collide_with_maze(maze, mouse.hitbox));
    mouse.translate(distance_to_far_wall, distance_to_far_wall);

    mouse.translate(distance_to_far_wall, -(distance_to_far_wall));
    CHECK(does_hitbox_collide_with_maze(maze, mouse.hitbox));
    mouse.translate(-(distance_to_far_wall), distance_to_far_wall);
}
