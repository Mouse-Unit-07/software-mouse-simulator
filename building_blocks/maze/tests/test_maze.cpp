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
#include <optional>
#include <utility>
#include "point.hpp"
#include "ray.hpp"
#include "rectangular_hitbox.hpp"
#include "maze.hpp"
#include "mouse.hpp"
#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>

using namespace maze;

/*============================================================================*/
/*                             Public Definitions                             */
/*============================================================================*/
constexpr double FLOAT_TOLERANCE{1e-6};

/* count all unique touches, including hitbox corners */
int count_touching_obstacles(const Maze& maze)
{
    const auto& obstacles{maze.obstacles};
    int touching_count{0};

    for (size_t i{0}; i < obstacles.size(); i++) {
        for (size_t j{i + 1}; j < obstacles.size(); j++) {
            const auto& a{obstacles.at(i)};
            const auto& b{obstacles.at(j)};

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
    Maze maze{build_maze_from_ascii(ascii, 0)};

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
    Maze maze{build_maze_from_ascii(ascii, 0)};
    const Cell& top_left {maze.get_cell(0, 0)};
    const Cell& bottom_right {maze.get_cell(1, 1)};

    CHECK(top_left.obstacles.size() == 8);
    CHECK(bottom_right.obstacles.size() == 7);
}

TEST(MazeTests, MouseStartCoordinatesCreated)
{
    std::vector<std::string> ascii
    {
        "+-+",
        "|S|",
        "+-+"
    };
    Maze maze{build_maze_from_ascii(ascii, 0)};

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
    Maze maze{build_maze_from_ascii(ascii, 0)};
    double expected_x{CELL_SIZE / 2.0};
    double expected_y{CELL_SIZE / 2.0};

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
    Maze maze{build_maze_from_ascii(ascii, 0)};
    int post_count{0};

    for (const auto& obstacle : maze.obstacles) {
        double width{obstacle.horizontal_size};
        double height{obstacle.vertical_size};

        if (fabs(width - OFFICIAL_POST_SIZE) < FLOAT_TOLERANCE &&
                fabs(height - OFFICIAL_POST_SIZE) < FLOAT_TOLERANCE) {
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
    Maze maze{build_maze_from_ascii(ascii, 0)};
    int wall_count{0};

    for (const auto& obstacle : maze.obstacles) {
        if (obstacle.vertical_size > obstacle.horizontal_size) {
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
    Maze maze{build_maze_from_ascii(ascii, 0)};
    int wall_count{0};

    for (const auto& obstacle : maze.obstacles) {
        if (obstacle.vertical_size < obstacle.horizontal_size) {
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
    Maze maze{build_maze_from_ascii(ascii, 0)};
    const Cell& left{maze.get_cell(0, 0)};
    const Cell& right{maze.get_cell(0, 1)};
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
    Maze maze{build_maze_from_ascii(ascii, 0)};
    const Cell& bottom{maze.get_cell(0, 0)};
    const Cell& top{maze.get_cell(1, 0)};
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
        "| |",
        "+-+"
    };
    Maze maze{build_maze_from_ascii(ascii, 0)};
    int touching_count{count_touching_obstacles(maze)};

    /* 8 full unique touches + 4 diagonal corner touches */
    CHECK(touching_count == 12);
}

TEST(MazeTests, WallAdjustmentsModifySizes)
{
    const double size_adjustment{1.07};

    std::vector<std::string> ascii
    {
        "+-+",
        "| |",
        "+-+"
    };
    Maze maze{build_maze_from_ascii(ascii, size_adjustment)};

    for (const auto& o : maze.obstacles) {
        if (o.horizontal_size > o.vertical_size) {
            DOUBLES_EQUAL(OFFICIAL_WALL_LENGTH_SIZE + size_adjustment, o.horizontal_size, FLOAT_TOLERANCE);
            DOUBLES_EQUAL(OFFICIAL_WALL_WIDTH_SIZE + size_adjustment, o.vertical_size, FLOAT_TOLERANCE);
        }

        if (o.vertical_size > o.horizontal_size) {
            DOUBLES_EQUAL(OFFICIAL_WALL_WIDTH_SIZE + size_adjustment, o.horizontal_size, FLOAT_TOLERANCE);
            DOUBLES_EQUAL(OFFICIAL_WALL_LENGTH_SIZE + size_adjustment, o.vertical_size, FLOAT_TOLERANCE);
        }
    }
}

TEST(MazeTests, PostAdjustmentsModifySizes)
{
    const double size_adjustment{1.07};

    std::vector<std::string> ascii
    {
        "+-+",
        "| |",
        "+-+"
    };
    Maze maze{build_maze_from_ascii(ascii, size_adjustment)};

    for (const auto& o : maze.obstacles) {
        if (fabs(o.horizontal_size - o.vertical_size) < FLOAT_TOLERANCE) {
            DOUBLES_EQUAL(OFFICIAL_POST_SIZE + size_adjustment, o.horizontal_size, FLOAT_TOLERANCE);
            DOUBLES_EQUAL(OFFICIAL_POST_SIZE + size_adjustment, o.vertical_size, FLOAT_TOLERANCE);
        }
    }
}

TEST(MazeTests, AdjustedWallsAndPostsTouch)
{
    const double size_adjustment{-1.07};

    std::vector<std::string> ascii
    {
        "+-+",
        "| |",
        "+-+"
    };
    Maze maze{build_maze_from_ascii(ascii, size_adjustment)};
    int touching_count{count_touching_obstacles(maze)};

    /* 8 full unique touches + 4 diagonal corner touches */
    CHECK(touching_count == 12);
}

TEST(MazeTests, MazeRowAndColumnComputableFromRawCoordinates)
{
    std::vector<std::string> ascii
    {
        "+-+ +",
        "|S|  ",
        "+-+-+",
        "  |  ",
        "+ +-+"
    };
    Maze maze{build_maze_from_ascii(ascii, 0.0)};

    auto test_1{get_cell_from_point(maze, maze.mouse_start)};
    auto test_2{get_cell_from_point(maze, geometry::Point{maze.mouse_start.x + CELL_SIZE, maze.mouse_start.y})};
    auto test_3{get_cell_from_point(maze, geometry::Point{maze.mouse_start.x, maze.mouse_start.y + CELL_SIZE})};
    auto test_4{get_cell_from_point(maze, geometry::Point{maze.mouse_start.x + CELL_SIZE, maze.mouse_start.y + CELL_SIZE})};

    CHECK(test_1.has_value());
    CHECK(test_2.has_value());
    CHECK(test_3.has_value());
    CHECK(test_4.has_value());

    auto [row_1, col_1] {*test_1};
    auto [row_2, col_2] {*test_2};
    auto [row_3, col_3] {*test_3};
    auto [row_4, col_4] {*test_4};
    CHECK((row_1 == 0) && (col_1 == 0));
    CHECK((row_2 == 0) && (col_2 == 1));
    CHECK((row_3 == 1) && (col_3 == 0));
    CHECK((row_4 == 1) && (col_4 == 1));
}

TEST(MazeTests, MazeRowAndColumnComputableFromAdjustedCoordinates)
{
    double adjustment{20.0};
    std::vector<std::string> ascii
    {
        "+-+ +",
        "|S|  ",
        "+-+-+",
        "  |  ",
        "+ +-+"
    };
    Maze maze{build_maze_from_ascii(ascii, adjustment)};

    double new_cell_size{CELL_SIZE + adjustment * 2};
    auto test_1{get_cell_from_point(maze, maze.mouse_start)};
    auto test_2{get_cell_from_point(maze, geometry::Point{maze.mouse_start.x + new_cell_size, maze.mouse_start.y})};
    auto test_3{get_cell_from_point(maze, geometry::Point{maze.mouse_start.x, maze.mouse_start.y + new_cell_size})};
    auto test_4{get_cell_from_point(maze, geometry::Point{maze.mouse_start.x + new_cell_size, maze.mouse_start.y + new_cell_size})};

    CHECK(test_1.has_value());
    CHECK(test_2.has_value());
    CHECK(test_3.has_value());
    CHECK(test_4.has_value());

    auto [row_1, col_1] {*test_1};
    auto [row_2, col_2] {*test_2};
    auto [row_3, col_3] {*test_3};
    auto [row_4, col_4] {*test_4};
    CHECK((row_1 == 0) && (col_1 == 0));
    CHECK((row_2 == 0) && (col_2 == 1));
    CHECK((row_3 == 1) && (col_3 == 0));
    CHECK((row_4 == 1) && (col_4 == 1));
}

TEST(MazeTests, NoMazeRowAndColumnFromOutOfBoundsCoordinates)
{
    std::vector<std::string> ascii
    {
        "+-+ +",
        "|S|  ",
        "+-+-+",
        "  |  ",
        "+ +-+"
    };
    Maze maze{build_maze_from_ascii(ascii, 0.0)};

    auto test_1{get_cell_from_point(maze, geometry::Point{maze.mouse_start.x - CELL_SIZE, maze.mouse_start.y})};
    auto test_2{get_cell_from_point(maze, geometry::Point{maze.mouse_start.x, maze.mouse_start.y + (CELL_SIZE * 2)})};

    CHECK(!test_1.has_value());
    CHECK(!test_2.has_value());
}

TEST(MazeTests, FourRayDistancesComputedInSingleCell)
{
    std::vector<std::string> ascii
    {
        "+-+",
        "|S|",
        "+-+"
    };
    Maze maze{build_maze_from_ascii(ascii, 0.0)};

    mouse::Mouse mouse;
    mouse.translate(maze.mouse_start.x, maze.mouse_start.y);

    for (int i{1}; i < 4; i++) {
        auto result_1{compute_ray_distance_in_vicinity(maze, mouse.ir_1_sensor, 0, 0)};
        auto result_2{compute_ray_distance_in_vicinity(maze, mouse.ir_2_sensor, 0, 0)};
        auto result_3{compute_ray_distance_in_vicinity(maze, mouse.ir_3_sensor, 0, 0)};
        auto result_4{compute_ray_distance_in_vicinity(maze, mouse.ir_4_sensor, 0, 0)};

        CHECK(result_1.has_value());
        CHECK(result_2.has_value());
        CHECK(result_3.has_value());
        CHECK(result_4.has_value());
        CHECK(*result_1 > 0.0);
        CHECK(*result_2 > 0.0);
        CHECK(*result_3 > 0.0);
        CHECK(*result_4 > 0.0);

        mouse.rotate((M_PI / 4) * i);
    }
}

TEST(MazeTests, FourRayDistancesComputedInClosedThreeByThree)
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
    Maze maze{build_maze_from_ascii(ascii, 0.0)};

    mouse::Mouse mouse;
    mouse.translate(maze.mouse_start.x, maze.mouse_start.y);

    auto mouse_row_and_column{get_cell_from_point(maze, maze.mouse_start)};
    auto [mouse_row, mouse_col]{*mouse_row_and_column};

    for (int i{1}; i < 4; i++) {
        auto result_1{compute_ray_distance_in_vicinity(maze, mouse.ir_1_sensor, mouse_row, mouse_col)};
        auto result_2{compute_ray_distance_in_vicinity(maze, mouse.ir_2_sensor, mouse_row, mouse_col)};
        auto result_3{compute_ray_distance_in_vicinity(maze, mouse.ir_3_sensor, mouse_row, mouse_col)};
        auto result_4{compute_ray_distance_in_vicinity(maze, mouse.ir_4_sensor, mouse_row, mouse_col)};

        CHECK(result_1.has_value());
        CHECK(result_2.has_value());
        CHECK(result_3.has_value());
        CHECK(result_4.has_value());
        CHECK(*result_1 > 0.0);
        CHECK(*result_2 > 0.0);
        CHECK(*result_3 > 0.0);
        CHECK(*result_4 > 0.0);

        mouse.rotate((M_PI / 4) * i);
    }
}

TEST(MazeTests, AllRayDistancesAreShortestDistance)
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
    Maze maze{build_maze_from_ascii(ascii, 0.0)};

    mouse::Mouse mouse;
    mouse.translate(maze.mouse_start.x, maze.mouse_start.y);

    auto mouse_row_and_column{get_cell_from_point(maze, maze.mouse_start)};
    auto [mouse_row, mouse_col]{*mouse_row_and_column};

    for (int i{1}; i < 4; i++) {
        auto result_1{compute_ray_distance_in_vicinity(maze, mouse.ir_1_sensor, mouse_row, mouse_col)};
        auto result_2{compute_ray_distance_in_vicinity(maze, mouse.ir_2_sensor, mouse_row, mouse_col)};
        auto result_3{compute_ray_distance_in_vicinity(maze, mouse.ir_3_sensor, mouse_row, mouse_col)};
        auto result_4{compute_ray_distance_in_vicinity(maze, mouse.ir_4_sensor, mouse_row, mouse_col)};

        CHECK(result_1.has_value());
        CHECK(result_2.has_value());
        CHECK(result_3.has_value());
        CHECK(result_4.has_value());
        CHECK(*result_1 < maze.cell_size);
        CHECK(*result_2 < maze.cell_size);
        CHECK(*result_3 < maze.cell_size);
        CHECK(*result_4 < maze.cell_size);

        mouse.rotate((M_PI / 4) * i);
    }
}

TEST(MazeTests, NoRayDistancesComputedInEmptyThreeByThree)
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
    Maze maze{build_maze_from_ascii(ascii, 0.0)};

    mouse::Mouse mouse;
    mouse.translate(maze.mouse_start.x, maze.mouse_start.y);

    auto mouse_row_and_column{get_cell_from_point(maze, maze.mouse_start)};
    auto [mouse_row, mouse_col]{*mouse_row_and_column};

    for (int i{1}; i < 4; i++) {
        auto result_1{compute_ray_distance_in_vicinity(maze, mouse.ir_1_sensor, mouse_row, mouse_col)};
        auto result_2{compute_ray_distance_in_vicinity(maze, mouse.ir_2_sensor, mouse_row, mouse_col)};
        auto result_3{compute_ray_distance_in_vicinity(maze, mouse.ir_3_sensor, mouse_row, mouse_col)};
        auto result_4{compute_ray_distance_in_vicinity(maze, mouse.ir_4_sensor, mouse_row, mouse_col)};

        CHECK(!(result_1.has_value()));
        CHECK(!(result_2.has_value()));
        CHECK(!(result_3.has_value()));
        CHECK(!(result_4.has_value()));

        mouse.rotate((M_PI / 4) * i);
    }
}

TEST(MazeTests, MouseAtCellCenterNoCollision)
{
    std::vector<std::string> ascii
    {
        "+-+-+-+",
        "| | | |",
        "+-+-+-+",
        "| |S| |",
        "+-+-+-+",
        "| | | |",
        "+-+-+-+",
    };
    Maze maze{build_maze_from_ascii(ascii, 0.0)};

    mouse::Mouse mouse;
    mouse.translate(maze.mouse_start.x, maze.mouse_start.y);

    auto [row, col]{*(get_cell_from_point(maze, maze.mouse_start))};

    CHECK(!does_hitbox_collide_in_vicinity(maze, mouse.hitbox, row, col));
}

TEST(MazeTests, MouseMovingNearWallsNoCollision)
{
    std::vector<std::string> ascii
    {
        "+-+",
        "|S|",
        "+-+"
    };
    Maze maze{build_maze_from_ascii(ascii, 0.0)};

    mouse::Mouse mouse;
    mouse.translate(maze.mouse_start.x, maze.mouse_start.y);

    double distance_to_wall_center{(OFFICIAL_POST_SIZE + OFFICIAL_WALL_LENGTH_SIZE) / 2};
    double horizontal_distance_to_cell_wall{(distance_to_wall_center - (mouse.hitbox.horizontal_size / 2) - (OFFICIAL_POST_SIZE / 2)) - 1};
    double vertical_distance_to_cell_wall{(distance_to_wall_center - (mouse.hitbox.vertical_size / 2) - (OFFICIAL_POST_SIZE / 2)) - 1};

    mouse.translate(0.0, vertical_distance_to_cell_wall);
    CHECK(!does_hitbox_collide_in_vicinity(maze, mouse.hitbox, 0.0, 0.0));
    mouse.translate(0.0, -vertical_distance_to_cell_wall);

    mouse.translate(-horizontal_distance_to_cell_wall, 0.0);
    CHECK(!does_hitbox_collide_in_vicinity(maze, mouse.hitbox, 0.0, 0.0));
    mouse.translate(horizontal_distance_to_cell_wall, 0.0);

    mouse.translate(-0.0, -vertical_distance_to_cell_wall);
    CHECK(!does_hitbox_collide_in_vicinity(maze, mouse.hitbox, 0.0, 0.0));
    mouse.translate(0.0, vertical_distance_to_cell_wall);

    mouse.translate(horizontal_distance_to_cell_wall, 0.0);
    CHECK(!does_hitbox_collide_in_vicinity(maze, mouse.hitbox, 0.0, 0.0));
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
    Maze maze{build_maze_from_ascii(ascii, 0.0)};

    mouse::Mouse mouse;
    mouse.translate(maze.mouse_start.x, maze.mouse_start.y);

    double distance_to_wall_center{(OFFICIAL_POST_SIZE + OFFICIAL_WALL_LENGTH_SIZE) / 2};
    double horizontal_distance_to_cell_wall{(distance_to_wall_center - (mouse.hitbox.horizontal_size / 2) - (OFFICIAL_POST_SIZE / 2))};
    double vertical_distance_to_cell_wall{(distance_to_wall_center - (mouse.hitbox.vertical_size / 2) - (OFFICIAL_POST_SIZE / 2))};

    mouse.translate(0.0, vertical_distance_to_cell_wall);
    CHECK(does_hitbox_collide_in_vicinity(maze, mouse.hitbox, 0.0, 0.0));
    mouse.translate(0.0, -vertical_distance_to_cell_wall);

    mouse.translate(-horizontal_distance_to_cell_wall, 0.0);
    CHECK(does_hitbox_collide_in_vicinity(maze, mouse.hitbox, 0.0, 0.0));
    mouse.translate(horizontal_distance_to_cell_wall, 0.0);

    mouse.translate(-0.0, -vertical_distance_to_cell_wall);
    CHECK(does_hitbox_collide_in_vicinity(maze, mouse.hitbox, 0.0, 0.0));
    mouse.translate(0.0, vertical_distance_to_cell_wall);

    mouse.translate(horizontal_distance_to_cell_wall, 0.0);
    CHECK(does_hitbox_collide_in_vicinity(maze, mouse.hitbox, 0.0, 0.0));
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
    Maze maze{build_maze_from_ascii(ascii, 0.0)};

    mouse::Mouse mouse;
    mouse.translate(maze.mouse_start.x, maze.mouse_start.y);

    double distance_to_near_wall{maze.cell_size / 2};

    mouse.translate(-(distance_to_near_wall), 0.0);
    CHECK(does_hitbox_collide_in_vicinity(maze, mouse.hitbox, 0, 0));
    mouse.translate(distance_to_near_wall, 0.0);

    mouse.translate(distance_to_near_wall, 0.0);
    CHECK(does_hitbox_collide_in_vicinity(maze, mouse.hitbox, 0, 0));
    mouse.translate(-(distance_to_near_wall), 0.0);

    mouse.translate(0.0, -(distance_to_near_wall));
    CHECK(does_hitbox_collide_in_vicinity(maze, mouse.hitbox, 0, 0));
    mouse.translate(0.0, distance_to_near_wall);

    mouse.translate(0.0, distance_to_near_wall);
    CHECK(does_hitbox_collide_in_vicinity(maze, mouse.hitbox, 0, 0));
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
    Maze maze{build_maze_from_ascii(ascii, 0.0)};

    mouse::Mouse mouse;
    mouse.translate(maze.mouse_start.x, maze.mouse_start.y);

    double distance_to_near_wall{maze.cell_size / 2};

    mouse.translate(distance_to_near_wall, distance_to_near_wall);
    CHECK(does_hitbox_collide_in_vicinity(maze, mouse.hitbox, 0, 0));
    mouse.translate(-(distance_to_near_wall), -(distance_to_near_wall));

    mouse.translate(-(distance_to_near_wall), distance_to_near_wall);
    CHECK(does_hitbox_collide_in_vicinity(maze, mouse.hitbox, 0, 0));
    mouse.translate(distance_to_near_wall, -(distance_to_near_wall));

    mouse.translate(-(distance_to_near_wall), -(distance_to_near_wall));
    CHECK(does_hitbox_collide_in_vicinity(maze, mouse.hitbox, 0, 0));
    mouse.translate(distance_to_near_wall, distance_to_near_wall);

    mouse.translate(distance_to_near_wall, -(distance_to_near_wall));
    CHECK(does_hitbox_collide_in_vicinity(maze, mouse.hitbox, 0, 0));
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
    Maze maze{build_maze_from_ascii(ascii, 0.0)};

    mouse::Mouse mouse;
    mouse.translate(maze.mouse_start.x, maze.mouse_start.y);

    double distance_to_far_wall{maze.cell_size + (maze.cell_size / 2)};

    mouse.translate(-(distance_to_far_wall), 0.0);
    CHECK(does_hitbox_collide_in_vicinity(maze, mouse.hitbox, 1, 1));
    mouse.translate(distance_to_far_wall, 0.0);

    mouse.translate(distance_to_far_wall, 0.0);
    CHECK(does_hitbox_collide_in_vicinity(maze, mouse.hitbox, 1, 1));
    mouse.translate(-(distance_to_far_wall), 0.0);

    mouse.translate(0.0, -(distance_to_far_wall));
    CHECK(does_hitbox_collide_in_vicinity(maze, mouse.hitbox, 1, 1));
    mouse.translate(0.0, distance_to_far_wall);

    mouse.translate(0.0, distance_to_far_wall);
    CHECK(does_hitbox_collide_in_vicinity(maze, mouse.hitbox, 1, 1));
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
    Maze maze{build_maze_from_ascii(ascii, 0.0)};

    mouse::Mouse mouse;
    mouse.translate(maze.mouse_start.x, maze.mouse_start.y);

    double distance_to_far_wall{maze.cell_size + (maze.cell_size / 2)};

    mouse.translate(distance_to_far_wall, distance_to_far_wall);
    CHECK(does_hitbox_collide_in_vicinity(maze, mouse.hitbox, 1, 1));
    mouse.translate(-(distance_to_far_wall), -(distance_to_far_wall));

    mouse.translate(-(distance_to_far_wall), distance_to_far_wall);
    CHECK(does_hitbox_collide_in_vicinity(maze, mouse.hitbox, 1, 1));
    mouse.translate(distance_to_far_wall, -(distance_to_far_wall));

    mouse.translate(-(distance_to_far_wall), -(distance_to_far_wall));
    CHECK(does_hitbox_collide_in_vicinity(maze, mouse.hitbox, 1, 1));
    mouse.translate(distance_to_far_wall, distance_to_far_wall);

    mouse.translate(distance_to_far_wall, -(distance_to_far_wall));
    CHECK(does_hitbox_collide_in_vicinity(maze, mouse.hitbox, 1, 1));
    mouse.translate(-(distance_to_far_wall), distance_to_far_wall);
}
