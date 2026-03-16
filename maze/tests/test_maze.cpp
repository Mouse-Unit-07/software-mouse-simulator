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
#include <vector>
#include <string>
#include "point.hpp"
#include "rectangular_hitbox.hpp"
#include "maze.hpp"
#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>

/*============================================================================*/
/*                             Public Definitions                             */
/*============================================================================*/
constexpr double FLOAT_TOLERANCE {1e-6};

int count_touching_obstacles(const maze::Maze& maze)
{
    const auto& obstacles {maze.obstacles};
    int touching_count {0};

    for (const auto& a : obstacles)
    {
        for (const auto& b : obstacles)
        {
            if (&a == &b) continue;

            double dx {fabs(a.center.x - b.center.x)};
            double dy {fabs(a.center.y - b.center.y)};

            double allowed_x {(a.horizontal_size / 2.0) + (b.horizontal_size / 2.0)};
            double allowed_y {(a.vertical_size / 2.0) + (b.vertical_size / 2.0)};

            bool touching_x {fabs(dx - allowed_x) < FLOAT_TOLERANCE};
            bool touching_y {fabs(dy - allowed_y) < FLOAT_TOLERANCE};

            if (touching_x && touching_y)
            {
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
    maze::Maze maze {maze::build_from_ascii(ascii, 0)};

    CHECK_EQUAL(2, maze.rows);
    CHECK_EQUAL(2, maze.cols);
}

TEST(MazeTests, CellCoordinatesAreTopLeftOrigin)
{
    std::vector<std::string> ascii =
    {
        "+-+ +",
        "|S|  ",
        "+-+-+",
        "  |  ",
        "+ +-+"
    };
    maze::Maze maze {maze::build_from_ascii(ascii, 0)};
    const maze::Cell& top_left {maze.get_cell(0, 0)};
    const maze::Cell& bottom_right {maze.get_cell(1, 1)};

    CHECK(top_left.obstacles.size() == 8);
    CHECK(bottom_right.obstacles.size() == 7);
}

TEST(MazeTests, MouseStartCoordinatesCreated)
{
    std::vector<std::string> ascii =
    {
        "+-+",
        "|S|",
        "+-+"
    };
    maze::Maze maze {maze::build_from_ascii(ascii, 0)};

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
    maze::Maze maze {maze::build_from_ascii(ascii, 0)};
    double expected_x {maze::CELL_SIZE / 2.0};
    double expected_y {maze::CELL_SIZE / 2.0};

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
    maze::Maze maze {maze::build_from_ascii(ascii, 0)};
    int post_count {0};

    for (const auto& obstacle : maze.obstacles)
    {
        double width {obstacle.horizontal_size};
        double height {obstacle.vertical_size};

        if (fabs(width - maze::OFFICIAL_POST_SIZE) < FLOAT_TOLERANCE &&
            fabs(height - maze::OFFICIAL_POST_SIZE) < FLOAT_TOLERANCE)
        {
            post_count++;
        }
    }

    CHECK_EQUAL(4, post_count);
}

TEST(MazeTests, CorrectNumberOfVerticalWalls)
{
    std::vector<std::string> ascii
    {
        "+ +",
        "|S|",
        "+ +"
    };
    maze::Maze maze {maze::build_from_ascii(ascii, 0)};
    int wall_count {0};

    for (const auto& obstacle : maze.obstacles)
    {
        if (obstacle.vertical_size >
            obstacle.horizontal_size)
        {
            wall_count++;
        }
    }

    CHECK(wall_count == 2);
}

TEST(MazeTests, CorrectNumberOfHorizontalWalls)
{
    std::vector<std::string> ascii
    {
        "+-+",
        " S ",
        "+-+"
    };
    maze::Maze maze {maze::build_from_ascii(ascii, 0)};
    int wall_count {0};

    for (const auto& obstacle : maze.obstacles)
    {
        if (obstacle.vertical_size <
            obstacle.horizontal_size)
        {
            wall_count++;
        }
    }

    CHECK(wall_count == 2);
}

TEST(MazeTests, VerticalWallSharedBetweenCells)
{
    std::vector<std::string> ascii
    {
        "+-+-+",
        "|S| |",
        "+-+-+"
    };
    maze::Maze maze {maze::build_from_ascii(ascii, 0)};
    const maze::Cell& left {maze.get_cell(0, 0)};
    const maze::Cell& right {maze.get_cell(0, 1)};
    bool shared {false};

    for (auto* a : left.obstacles)
    {
        for (auto* b : right.obstacles)
        {
            if (a == b)
            {
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
    maze::Maze maze {maze::build_from_ascii(ascii, 0)};
    const maze::Cell& bottom {maze.get_cell(0, 0)};
    const maze::Cell& top {maze.get_cell(1, 0)};
    bool shared {false};

    for (auto* a : bottom.obstacles)
    {
        for (auto* b : top.obstacles)
        {
            if (a == b)
            {
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
        "| |",
        "+-+"
    };
    maze::Maze maze {maze::build_from_ascii(ascii, 0)};
    int touching_count {count_touching_obstacles(maze)};

    CHECK(touching_count == 8);
}

TEST(MazeTests, WallAdjustmentsModifySizes)
{
    const double size_adjustment {1.07};

    std::vector<std::string> ascii
    {
        "+-+",
        "| |",
        "+-+"
    };
    maze::Maze maze {maze::build_from_ascii(ascii, size_adjustment)};

    for (const auto& o : maze.obstacles)
    {
        if (o.horizontal_size > o.vertical_size) {
            DOUBLES_EQUAL(maze::OFFICIAL_WALL_LENGTH_SIZE + size_adjustment, o.horizontal_size, FLOAT_TOLERANCE);
            DOUBLES_EQUAL(maze::OFFICIAL_WALL_WIDTH_SIZE + size_adjustment, o.vertical_size, FLOAT_TOLERANCE);
        }

        if (o.vertical_size > o.horizontal_size)
        {
            DOUBLES_EQUAL(maze::OFFICIAL_WALL_WIDTH_SIZE + size_adjustment, o.horizontal_size, FLOAT_TOLERANCE);
            DOUBLES_EQUAL(maze::OFFICIAL_WALL_LENGTH_SIZE + size_adjustment, o.vertical_size, FLOAT_TOLERANCE);
        }
    }
}

TEST(MazeTests, PostAdjustmentsModifySizes)
{
    const double size_adjustment {1.07};

    std::vector<std::string> ascii
    {
        "+-+",
        "| |",
        "+-+"
    };
    maze::Maze maze {maze::build_from_ascii(ascii, size_adjustment)};

    for (const auto& o : maze.obstacles)
    {
        if (fabs(o.horizontal_size - o.vertical_size) < FLOAT_TOLERANCE)
        {
            DOUBLES_EQUAL(maze::OFFICIAL_POST_SIZE + size_adjustment, o.horizontal_size, FLOAT_TOLERANCE);
            DOUBLES_EQUAL(maze::OFFICIAL_POST_SIZE + size_adjustment, o.vertical_size, FLOAT_TOLERANCE);
        }
    }
}

TEST(MazeTests, AdjustedWallsAndPostsTouch)
{
    const double size_adjustment {-1.07};

    std::vector<std::string> ascii
    {
        "+-+",
        "| |",
        "+-+"
    };
    maze::Maze maze {maze::build_from_ascii(ascii, size_adjustment)};
    int touching_count {count_touching_obstacles(maze)};

    CHECK(touching_count == 8);
}
