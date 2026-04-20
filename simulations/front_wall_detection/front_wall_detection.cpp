/*-------------------------------- FILE INFO ---------------------------------*/
/* Filename           : front_wall_detection.cpp                              */
/*                                                                            */
/* Implementation for micromouse front wall detection simulation and          */
/* associated config and results analysis helpers                             */
/*                                                                            */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*                               Include Files                                */
/*----------------------------------------------------------------------------*/
extern "C"
{

#include <math.h>
#include <stdint.h>
#include "mock_device_drivers.h"
#include "infrared_sensor.h"

}

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <map>
#include <random>
#include <sstream>
#include <string>
#include <vector>
#include "point.hpp"
#include "ray.hpp"
#include "rectangular_hitbox.hpp"
#include "mouse.hpp"
#include "maze.hpp"
#include "visualizer.hpp"
#include "simulation_common.hpp"
#include "front_wall_detection.hpp"

/*----------------------------------------------------------------------------*/
/*                            Private Declarations                            */
/*----------------------------------------------------------------------------*/
namespace
{

using namespace front_wall_detection;

void prepare_mock_for_front_wall_detection(const Config& cfg, const maze::Maze& maze,
                                           mouse::Mouse& mouse);

} /* unnamed namespace */

/*----------------------------------------------------------------------------*/
/*                               Private Globals                              */
/*----------------------------------------------------------------------------*/
namespace
{

const std::string TEST_OUTPUT_DIRECTORY{"front-wall-detection-visualizer"};
bool visualizer_enabled{false};
visualizer::Visualizer wall_absent_visualizer;
visualizer::Visualizer wall_present_visualizer;
ControlConfig ctr_lower_bounds{};
ControlConfig ctr_upper_bounds{};
EnvironmentConfig env_lower_bounds{};
EnvironmentConfig env_upper_bounds{};

} /* unnamed namespace */

/*----------------------------------------------------------------------------*/
/*                             Public Definitions                             */
/*----------------------------------------------------------------------------*/
namespace front_wall_detection
{

void reset_all_config_bounds(void)
{
    ctr_lower_bounds = {};
    ctr_upper_bounds = {};
    env_lower_bounds = {};
    env_upper_bounds = {};
}

void set_ctr_config_bounds(const ControlConfig& lower, const ControlConfig& upper)
{
    ctr_lower_bounds = lower;
    ctr_upper_bounds = upper;
}

void set_env_config_bounds(const EnvironmentConfig& lower, const EnvironmentConfig& upper)
{
    env_lower_bounds = lower;
    env_upper_bounds = upper;
}

ControlConfig decode_control(const std::vector<double>& x)
{
    ControlConfig c{};
    size_t i{0};

    c.reading_threshold = static_cast<uint32_t>(x.at(i++));

    return c;
}

std::vector<double> encode_control(const ControlConfig& cfg)
{
    return {static_cast<double>(cfg.reading_threshold)};
}

std::pair<std::vector<double>, std::vector<double>> get_control_bounds(void)
{
    return {encode_control(ctr_lower_bounds), encode_control(ctr_upper_bounds)};
}

EnvironmentConfig generate_random_environment(void)
{
    static thread_local std::mt19937 rng(std::random_device{}());

    auto uniform = [&](double a, double b) {
        return std::uniform_real_distribution<double>(a, b)(rng);
    };

    EnvironmentConfig e;
    e.ir_reading_scale =
        uniform(env_lower_bounds.ir_reading_scale, env_upper_bounds.ir_reading_scale);
    e.mouse_angle =
        uniform(env_lower_bounds.mouse_angle, env_upper_bounds.mouse_angle);
    e.horizontal_position_variance =
        uniform(env_lower_bounds.horizontal_position_variance, env_upper_bounds.horizontal_position_variance);
    e.vertical_position_variance =
        uniform(env_lower_bounds.vertical_position_variance, env_upper_bounds.vertical_position_variance);

    return e;
}

void enable_visualization(void)
{
    visualizer_enabled = true;
}

void disable_visualization(void)
{
    visualizer_enabled = false;
}

std::string config_to_string(const Config& cfg)
{
    std::ostringstream oss;

    oss << simulation_common::double_to_filename(cfg.env_cfg.ir_reading_scale) << "-"
        << simulation_common::double_to_filename(cfg.env_cfg.mouse_angle) << "-"
        << simulation_common::double_to_filename(cfg.env_cfg.horizontal_position_variance) << "-"
        << simulation_common::double_to_filename(cfg.env_cfg.vertical_position_variance) << "-"
        << simulation_common::double_to_filename(cfg.ctrl_cfg.reading_threshold);

    return oss.str();
}

Result run_simulation(const Config& cfg)
{
    if (visualizer_enabled) {
        std::filesystem::create_directories(TEST_OUTPUT_DIRECTORY);
    }
    std::vector<std::string> ascii_open{
        "+-+",
        "|S|",
        "+ +",
        "| |",
        "+-+"
    };
    maze::Maze open_maze{maze::build_maze_from_ascii(ascii_open, 0.0)};

    std::vector<std::string> ascii_closed{
        "+-+",
        "|S|",
        "+-+",
        "   ",
        "   "
    };
    maze::Maze closed_maze{maze::build_maze_from_ascii(ascii_closed, 0.0)};

    mouse::Mouse mouse;
    prepare_mock_for_front_wall_detection(cfg, open_maze, mouse);

    if (visualizer_enabled) {
        wall_absent_visualizer.draw_maze(100.0f, open_maze);
        wall_present_visualizer.draw_maze(100.0f, closed_maze);
        wall_absent_visualizer.draw_mouse_on_maze(mouse);
        wall_present_visualizer.draw_mouse_on_maze(mouse);
    }

    bool identified_absent_wall{false};
    bool identified_present_wall{false};
    uint32_t ir_1_reading{0u};
    uint32_t ir_4_reading{0u};
    uint32_t average_reading{0u};
    double ir_1_distance{0.0};
    double ir_4_distance{0.0};

    ir_1_distance = maze::compute_ray_distance_in_closed_space(open_maze, mouse.hitbox.center,
                                                               mouse.ir_1_sensor);
    ir_4_distance = maze::compute_ray_distance_in_closed_space(open_maze, mouse.hitbox.center,
                                                               mouse.ir_4_sensor);
    update_ir_1_sensor_reading(ir_1_distance);
    update_ir_4_sensor_reading(ir_4_distance);
    ir_1_reading = read_ir_1_sensor();
    ir_1_reading = scale_and_clamp_ir_sensor_reading(ir_1_reading, cfg.env_cfg.ir_reading_scale);
    ir_4_reading = read_ir_4_sensor();
    ir_4_reading = scale_and_clamp_ir_sensor_reading(ir_4_reading, cfg.env_cfg.ir_reading_scale);
    average_reading = (ir_1_reading + ir_4_reading) / 2;
    identified_absent_wall = (average_reading < cfg.ctrl_cfg.reading_threshold) ? true : false;

    if (visualizer_enabled) {
        if (identified_absent_wall) {
            wall_absent_visualizer.change_beam_color_to_red();
        }
        wall_absent_visualizer.draw_ir_1_sensor_beam(mouse, ir_1_distance);
        wall_absent_visualizer.draw_ir_4_sensor_beam(mouse, ir_4_distance);
        wall_absent_visualizer.reset_beam_color();
    }

    ir_1_distance = maze::compute_ray_distance_in_closed_space(closed_maze, mouse.hitbox.center,
                                                               mouse.ir_1_sensor);
    ir_4_distance = maze::compute_ray_distance_in_closed_space(closed_maze, mouse.hitbox.center,
                                                               mouse.ir_4_sensor);
    update_ir_1_sensor_reading(ir_1_distance);
    update_ir_4_sensor_reading(ir_4_distance);
    ir_1_reading = read_ir_1_sensor();
    ir_1_reading = scale_and_clamp_ir_sensor_reading(ir_1_reading, cfg.env_cfg.ir_reading_scale);
    ir_4_reading = read_ir_4_sensor();
    ir_4_reading = scale_and_clamp_ir_sensor_reading(ir_4_reading, cfg.env_cfg.ir_reading_scale);
    average_reading = (ir_1_reading + ir_4_reading) / 2;
    identified_present_wall = (average_reading >= cfg.ctrl_cfg.reading_threshold) ? true : false;

    if (visualizer_enabled) {
        if (identified_present_wall) {
            wall_present_visualizer.change_beam_color_to_red();
        }
        wall_present_visualizer.draw_ir_1_sensor_beam(mouse, ir_1_distance);
        wall_present_visualizer.draw_ir_4_sensor_beam(mouse, ir_4_distance);
        wall_present_visualizer.reset_beam_color();
    }

    if (visualizer_enabled) {
        wall_absent_visualizer.save_to_image_file(TEST_OUTPUT_DIRECTORY + "/"
                                                  + config_to_string(cfg) + "-wa.png");
        wall_present_visualizer.save_to_image_file(TEST_OUTPUT_DIRECTORY + "/"
                                                   + config_to_string(cfg) + "-wp.png");
    }

    return Result{identified_absent_wall, identified_present_wall};
}

} /* front_wall_detection namespace */

/*----------------------------------------------------------------------------*/
/*                             Private Definitions                            */
/*----------------------------------------------------------------------------*/
namespace
{

using namespace front_wall_detection;

void prepare_mock_for_front_wall_detection(const Config& cfg, const maze::Maze& maze,
                                           mouse::Mouse& mouse)
{
    reset_mock_device_drivers();

    double max_horizontal_offset{(maze::OFFICIAL_WALL_LENGTH_SIZE - mouse.hitbox.horizontal_size)
                                 / 2};
    double max_vertical_offset{(maze::OFFICIAL_WALL_LENGTH_SIZE - mouse.hitbox.vertical_size) / 2};

    mouse.rotate(cfg.env_cfg.mouse_angle);
    mouse.translate(
        maze.mouse_start.x + (max_horizontal_offset * cfg.env_cfg.horizontal_position_variance),
        maze.mouse_start.y + (max_vertical_offset * cfg.env_cfg.vertical_position_variance));
}

} /* unnamed namespace */
