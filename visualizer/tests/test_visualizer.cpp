/*================================ FILE INFO =================================*/
/* Filename           : test_visualizer.cpp                                   */
/*                                                                            */
/* Test implementation for visualizer.cpp                                     */
/*                                                                            */
/*============================================================================*/

/*============================================================================*/
/*                               Include Files                                */
/*============================================================================*/
#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>
#include <cmath>
#include <filesystem>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include "point.hpp"
#include "ray.hpp"
#include "rectangular_hitbox.hpp"
#include "mouse.hpp"
#include "maze.hpp"
#include "visualizer.hpp"

using namespace visualizer;

/*============================================================================*/
/*                             Public Definitions                             */
/*============================================================================*/
const std::string TEST_OUTPUT_DIRECTORY{"visualizer-test-images"};

void create_test_images_directory(void)
{
    std::filesystem::create_directories(TEST_OUTPUT_DIRECTORY);
}

void draw_mouse_sensor_beams_to_nearest_walls(visualizer::Visualizer& visualizer, maze::Maze& maze,
                                              const mouse::Mouse mouse)
{
    visualizer.draw_maze(100.0f, maze);
    visualizer.draw_mouse_on_maze(mouse);
    double ir_1_distance{
        maze::compute_ray_distance_in_closed_space(maze, mouse.hitbox.center, mouse.ir_1_sensor)};
    double ir_2_distance{
        maze::compute_ray_distance_in_closed_space(maze, mouse.hitbox.center, mouse.ir_2_sensor)};
    double ir_3_distance{
        maze::compute_ray_distance_in_closed_space(maze, mouse.hitbox.center, mouse.ir_3_sensor)};
    double ir_4_distance{
        maze::compute_ray_distance_in_closed_space(maze, mouse.hitbox.center, mouse.ir_4_sensor)};
    visualizer.draw_ir_1_sensor_beam(mouse, ir_1_distance);
    visualizer.draw_ir_2_sensor_beam(mouse, ir_2_distance);
    visualizer.draw_ir_3_sensor_beam(mouse, ir_3_distance);
    visualizer.draw_ir_4_sensor_beam(mouse, ir_4_distance);
}

/*============================================================================*/
/*                            Mock Implementations                            */
/*============================================================================*/
/* none */

/*============================================================================*/
/*                                 Test Group                                 */
/*============================================================================*/
TEST_GROUP(VisualizerTests)
{
    void setup() override
    {
        create_test_images_directory();
    }

    void teardown() override
    {
        
    }
};

/*============================================================================*/
/*                                    Tests                                   */
/*============================================================================*/
IGNORE_TEST(VisualizerTests, DrawEmptyMaze)
{
    Visualizer visualizer;
    maze::Maze maze;
    maze.rows = 4;
    maze.cols = 4;

    visualizer.draw_maze(100.0f, maze);
    visualizer.save_to_image_file(TEST_OUTPUT_DIRECTORY + "/draw-empty-maze.png");
}

IGNORE_TEST(VisualizerTests, DrawMazeWithObstacles)
{
    Visualizer visualizer;
    std::vector<std::string> ascii
    {
        "+-+ +",
        "|S|  ",
        "+-+-+",
        "  |  ",
        "+ +-+"
    };
    maze::Maze maze{maze::build_maze_from_ascii(ascii, 0)};
    
    visualizer.draw_maze(100.0f, maze);
    visualizer.save_to_image_file(TEST_OUTPUT_DIRECTORY + "/draw-maze-with-obstacles.png");
}

IGNORE_TEST(VisualizerTests, DrawFullMaze)
{
    Visualizer visualizer;
    std::vector<std::string> ascii
    {
        "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+",
        "| | | | | | | | | | | | | | | | |",
        "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+",
        "| | | | | | | | | | | | | | | | |",
        "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+",
        "| | | | | | | | | | | | | | | | |",
        "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+",
        "| | | | | | | | | | | | | | | | |",
        "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+",
        "| | | | | | | | | | | | | | | | |",
        "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+",
        "| | | | | | | | | | | | | | | | |",
        "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+",
        "| | | | | | | | | | | | | | | | |",
        "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+",
        "| | | | | | | | | | | | | | | | |",
        "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+",
        "| | | | | | | | | | | | | | | | |",
        "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+",
        "| | | | | | | | | | | | | | | | |",
        "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+",
        "| | | | | | | | | | | | | | | | |",
        "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+",
        "| | | | | | | | | | | | | | | | |",
        "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+",
        "| | | | | | | | | | | | | | | | |",
        "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+",
        "| | | | | | | | | | | | | | | | |",
        "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+",
        "|S| | | | | | | | | | | | | | | |",
        "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+"
    };
    maze::Maze maze{maze::build_maze_from_ascii(ascii, 0)};

    visualizer.draw_maze(40.0f, maze);
    visualizer.save_to_image_file(TEST_OUTPUT_DIRECTORY + "/draw-full-maze.png");
}

IGNORE_TEST(VisualizerTests, DrawMouseOnMaze)
{
    Visualizer visualizer;
    std::vector<std::string> ascii
    {
        "+-+ +",
        "|S|  ",
        "+-+-+",
        "  |  ",
        "+ +-+"
    };
    maze::Maze maze{maze::build_maze_from_ascii(ascii, 0)};
    mouse::Mouse mouse;
    mouse.translate(maze.mouse_start.x, maze.mouse_start.y);
    
    visualizer.draw_maze(100.0f, maze);
    visualizer.draw_mouse_on_maze(mouse);
    visualizer.save_to_image_file(TEST_OUTPUT_DIRECTORY + "/draw-mouse-on-maze.png");
}

IGNORE_TEST(VisualizerTests, DrawMouseSensorBeams)
{
    Visualizer visualizer;
    std::vector<std::string> ascii
    {
        "+-+ +",
        "|S|  ",
        "+-+-+",
        "  |  ",
        "+ +-+"
    };
    maze::Maze maze{maze::build_maze_from_ascii(ascii, 0)};
    mouse::Mouse mouse;
    mouse.translate(maze.mouse_start.x, maze.mouse_start.y);
    
    visualizer.draw_maze(100.0f, maze);
    visualizer.draw_mouse_on_maze(mouse);
    visualizer.draw_ir_1_sensor_beam(mouse, 180.0);
    visualizer.draw_ir_2_sensor_beam(mouse, 180.0);
    visualizer.draw_ir_3_sensor_beam(mouse, 180.0);
    visualizer.draw_ir_4_sensor_beam(mouse, 180.0);
    visualizer.save_to_image_file(TEST_OUTPUT_DIRECTORY + "/draw-mouse-sensor-beams.png");
}

IGNORE_TEST(VisualizerTests, ChangeAndResetBeamColor)
{
    Visualizer visualizer;
    std::vector<std::string> ascii
    {
        "+-+ +",
        "|S|  ",
        "+-+-+",
        "  |  ",
        "+ +-+"
    };
    maze::Maze maze{maze::build_maze_from_ascii(ascii, 0)};
    mouse::Mouse mouse;
    mouse.translate(maze.mouse_start.x, maze.mouse_start.y);

    visualizer.draw_maze(100.0f, maze);
    visualizer.draw_mouse_on_maze(mouse);
    visualizer.change_beam_color_to_red();
    visualizer.draw_ir_1_sensor_beam(mouse, 180.0);
    visualizer.reset_beam_color();
    visualizer.draw_ir_2_sensor_beam(mouse, 180.0);
    visualizer.change_beam_color_to_red();
    visualizer.draw_ir_3_sensor_beam(mouse, 180.0);
    visualizer.reset_beam_color();
    visualizer.draw_ir_4_sensor_beam(mouse, 180.0);
    visualizer.save_to_image_file(TEST_OUTPUT_DIRECTORY + "/draw-red-and-regular-beams.png");
}

IGNORE_TEST(VisualizerTests, ChangeAndResetMouseColor)
{
    Visualizer visualizer;
    std::vector<std::string> ascii
    {
        "+-+",
        "|S|",
        "+-+"
    };
    maze::Maze maze{maze::build_maze_from_ascii(ascii, 0)};
    mouse::Mouse mouse;
    mouse.translate(maze.mouse_start.x, maze.mouse_start.y);

    visualizer.draw_maze(100.0f, maze);
    visualizer.draw_mouse_on_maze(mouse);

    visualizer.change_mouse_color_to_green();
    mouse.rotate(M_PI / 6);
    visualizer.draw_mouse_on_maze(mouse);

    visualizer.change_mouse_color_to_blue();
    mouse.rotate(M_PI / 6);
    visualizer.draw_mouse_on_maze(mouse);

    visualizer.reset_mouse_color();
    mouse.rotate(M_PI / 6);
    visualizer.draw_mouse_on_maze(mouse);

    visualizer.save_to_image_file(TEST_OUTPUT_DIRECTORY + "/draw-green-blue-and-regular-mouse.png");
}

IGNORE_TEST(VisualizerTests, DrawMouseSensorBeamsToNearestWalls)
{
    Visualizer visualizer;
    std::vector<std::string> ascii
    {
        "+-+-+-+",
        "| | | |",
        "+-+-+-+",
        "| |S  |",
        "+-+ +-+",
        "| | | |",
        "+-+-+-+"
    };
    maze::Maze maze{maze::build_maze_from_ascii(ascii, 0)};
    mouse::Mouse mouse;
    mouse.translate(maze.mouse_start.x, maze.mouse_start.y);
    mouse.rotate(-M_PI / 4);

    draw_mouse_sensor_beams_to_nearest_walls(visualizer, maze, mouse);

    visualizer.save_to_image_file(TEST_OUTPUT_DIRECTORY
                                  + "/draw-mouse-sensor-beams-to-nearest-walls.png");
}

IGNORE_TEST(VisualizerTests, DrawMouseAndBeamsOnLargeScaledMaze)
{
    Visualizer visualizer;
    std::vector<std::string> ascii
    {
        "+-+-+-+",
        "| | | |",
        "+-+-+-+",
        "| |S  |",
        "+-+ +-+",
        "| | | |",
        "+-+-+-+"
    };
    maze::Maze maze{maze::build_maze_from_ascii(ascii, 5)};
    mouse::Mouse mouse;
    mouse.translate(maze.mouse_start.x, maze.mouse_start.y);
    mouse.rotate(-M_PI / 4);

    draw_mouse_sensor_beams_to_nearest_walls(visualizer, maze, mouse);

    visualizer.save_to_image_file(TEST_OUTPUT_DIRECTORY
                                  + "/draw-mouse-and-beams-on-large-scale-maze.png");
}

IGNORE_TEST(VisualizerTests, DrawMouseAndBeamsOnSmallScaledMaze)
{
    Visualizer visualizer;
    std::vector<std::string> ascii
    {
        "+-+-+-+",
        "| | | |",
        "+-+-+-+",
        "| |S  |",
        "+-+ +-+",
        "| | | |",
        "+-+-+-+"
    };
    maze::Maze maze{maze::build_maze_from_ascii(ascii, 0.2)};
    mouse::Mouse mouse;
    mouse.translate(maze.mouse_start.x, maze.mouse_start.y);
    mouse.rotate(-M_PI / 4);

    draw_mouse_sensor_beams_to_nearest_walls(visualizer, maze, mouse);

    visualizer.save_to_image_file(TEST_OUTPUT_DIRECTORY
                                  + "/draw-mouse-and-beams-on-small-scale-maze.png");
}
