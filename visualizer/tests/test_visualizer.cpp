/*================================ FILE INFO =================================*/
/* Filename           : test_visualizer.cpp                                   */
/*                                                                            */
/* Test implementation for visualizer.cpp                                     */
/*                                                                            */
/*============================================================================*/

/*============================================================================*/
/*                               Include Files                                */
/*============================================================================*/
#include <cmath>
#include <vector>
#include <string>
#include <SFML/Graphics.hpp>
#include "point.hpp"
#include "ray.hpp"
#include "rectangular_hitbox.hpp"
#include "mouse.hpp"
#include "maze.hpp"
#include "visualizer.hpp"

#include <filesystem>
#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>

/*============================================================================*/
/*                             Public Definitions                             */
/*============================================================================*/
const std::string TEST_OUTPUT_DIRECTORY {"visualizer-test-images"};

void create_test_images_directory(void)
{
    std::filesystem::create_directories(TEST_OUTPUT_DIRECTORY);
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
TEST(VisualizerTests, DrawEmptyMaze)
{
    visualizer::Visualizer visualizer;
    maze::Maze maze;
    maze.rows = 4;
    maze.cols = 4;

    visualizer.draw_maze(100.0f, maze);
    visualizer.save_to_image_file(TEST_OUTPUT_DIRECTORY + "/draw-empty-maze.png");
}

TEST(VisualizerTests, DrawMazeWithObstacles)
{
    visualizer::Visualizer visualizer;
    std::vector<std::string> ascii
    {
        "+-+ +",
        "|S|  ",
        "+-+-+",
        "  |  ",
        "+ +-+"
    };
    maze::Maze maze {maze::build_from_ascii(ascii, 0)};
    
    visualizer.draw_maze(100.0f, maze);
    visualizer.save_to_image_file(TEST_OUTPUT_DIRECTORY + "/draw-maze-with-obstacles.png");
}

TEST(VisualizerTests, DrawFullMaze)
{
    visualizer::Visualizer visualizer;
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
    maze::Maze maze {maze::build_from_ascii(ascii, 0)};

    visualizer.draw_maze(40.0f, maze);
    visualizer.save_to_image_file(TEST_OUTPUT_DIRECTORY + "/draw-full-maze.png");
}

TEST(VisualizerTests, DrawMouseOnMaze)
{
    visualizer::Visualizer visualizer;
    std::vector<std::string> ascii
    {
        "+-+ +",
        "|S|  ",
        "+-+-+",
        "  |  ",
        "+ +-+"
    };
    maze::Maze maze {maze::build_from_ascii(ascii, 0)};
    mouse::Mouse mouse;
    mouse.translate(maze.mouse_start.x, maze.mouse_start.y);
    
    visualizer.draw_maze(100.0f, maze);
    visualizer.draw_mouse_on_maze(mouse);
    visualizer.save_to_image_file(TEST_OUTPUT_DIRECTORY + "/draw-mouse-on-maze.png");
}

TEST(VisualizerTests, DrawMouseSensorBeams)
{
    visualizer::Visualizer visualizer;
    std::vector<std::string> ascii
    {
        "+-+ +",
        "|S|  ",
        "+-+-+",
        "  |  ",
        "+ +-+"
    };
    maze::Maze maze {maze::build_from_ascii(ascii, 0)};
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
