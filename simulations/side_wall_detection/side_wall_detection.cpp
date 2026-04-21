/*-------------------------------- FILE INFO ---------------------------------*/
/* Filename           : side_wall_detection.cpp                               */
/*                                                                            */
/* Implementation for micromouse side wall detection simulation and           */
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
#include <deque>
#include <filesystem>
#include <functional>
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
#include "side_wall_detection.hpp"

/*----------------------------------------------------------------------------*/
/*                            Private Declarations                            */
/*----------------------------------------------------------------------------*/
namespace
{

using namespace side_wall_detection;

void prepare_mock_for_side_wall_detection(const Config& cfg, const maze::Maze& maze,
                                          mouse::Mouse& mouse);

} /* unnamed namespace */

/*----------------------------------------------------------------------------*/
/*                               Private Globals                              */
/*----------------------------------------------------------------------------*/
namespace
{

const std::string TEST_OUTPUT_DIRECTORY{"wall-detection-visualizer"};
std::string TEST_OUTPUT_SUBDIRECTORY{""};
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
namespace side_wall_detection
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
    e.maze_size_scale =
        uniform(env_lower_bounds.maze_size_scale, env_upper_bounds.maze_size_scale);
    e.ir_reading_scale =
        uniform(env_lower_bounds.ir_reading_scale, env_upper_bounds.ir_reading_scale);
    e.mouse_angle =
        uniform(env_lower_bounds.mouse_angle, env_upper_bounds.mouse_angle);
    e.horizontal_position_variance =
        uniform(env_lower_bounds.horizontal_position_variance, env_upper_bounds.horizontal_position_variance);
    e.vertical_position_variance =
        uniform(env_lower_bounds.vertical_position_variance, env_upper_bounds.vertical_position_variance);
    e.total_steps =
        uniform(env_lower_bounds.total_steps, env_upper_bounds.total_steps);

    return e;
}

void enable_visualization(const std::string& foldername)
{
    TEST_OUTPUT_SUBDIRECTORY = foldername;
    visualizer_enabled = true;
}

void disable_visualization(void)
{
    visualizer_enabled = false;
}

std::string config_to_string(const Config& cfg)
{
    std::ostringstream oss;

    oss << simulation_common::double_to_filename(cfg.env_cfg.maze_size_scale) << "-"
        << simulation_common::double_to_filename(cfg.env_cfg.ir_reading_scale) << "-"
        << simulation_common::double_to_filename(cfg.env_cfg.mouse_angle) << "-"
        << simulation_common::double_to_filename(cfg.env_cfg.horizontal_position_variance) << "-"
        << simulation_common::double_to_filename(cfg.env_cfg.vertical_position_variance) << "-"
        << cfg.env_cfg.total_steps << "-"
        << simulation_common::double_to_filename(cfg.ctrl_cfg.reading_threshold);

    return oss.str();
}

Result run_simulation(const Config& cfg)
{
    if (visualizer_enabled) {
        std::filesystem::create_directories(TEST_OUTPUT_DIRECTORY + "/" + TEST_OUTPUT_SUBDIRECTORY);
    }

    /* create maze */
    std::vector<std::string> ascii_open{
        "  +-+",
        "  |S|",
        "+-+ +",
        "|   |",
        "+-+-+"
    };
    maze::Maze open_maze{maze::build_maze_from_ascii(ascii_open, maze::OFFICIAL_POST_SIZE * (cfg.env_cfg.maze_size_scale - 1))};

    std::vector<std::string> ascii_closed{
        "  +-+",
        "  |S|",
        "  + +",
        "  | |",
        "  +-+"
    };
    maze::Maze closed_maze{maze::build_maze_from_ascii(ascii_closed, maze::OFFICIAL_POST_SIZE * (cfg.env_cfg.maze_size_scale - 1))};

    /* prepare mouse for wall detection */
    mouse::Mouse mouse;
    prepare_mock_for_side_wall_detection(cfg, open_maze, mouse);

    if (visualizer_enabled) {
        wall_absent_visualizer.draw_maze(100.0f, open_maze);
        wall_present_visualizer.draw_maze(100.0f, closed_maze);
        wall_absent_visualizer.draw_mouse_on_maze(mouse);
        wall_present_visualizer.draw_mouse_on_maze(mouse);
    }

    std::vector<bool> wall_absent_at_step;
    std::vector<bool> wall_present_at_step;
    wall_absent_at_step.resize(cfg.env_cfg.total_steps);
    wall_present_at_step.resize(cfg.env_cfg.total_steps);

    double open_maze_distance{0};
    double closed_maze_distance{0};
    uint32_t reading{0u};

    for (int i{0}; i < cfg.env_cfg.total_steps; i++) {
        open_maze_distance = maze::compute_ray_distance_in_closed_space(
            open_maze, mouse.hitbox.center, mouse.ir_3_sensor);
        update_ir_3_sensor_reading(open_maze_distance);
        reading =
            scale_and_clamp_ir_sensor_reading(read_ir_3_sensor(), cfg.env_cfg.ir_reading_scale);
        wall_absent_at_step.at(i) = (reading < cfg.ctrl_cfg.reading_threshold) ? true : false;

        closed_maze_distance = maze::compute_ray_distance_in_closed_space(
            closed_maze, mouse.hitbox.center, mouse.ir_3_sensor);
        update_ir_3_sensor_reading(closed_maze_distance);
        reading =
            scale_and_clamp_ir_sensor_reading(read_ir_3_sensor(), cfg.env_cfg.ir_reading_scale);
        wall_present_at_step.at(i) = (reading >= cfg.ctrl_cfg.reading_threshold) ? true : false;

        if (visualizer_enabled) {
            if (wall_absent_at_step.at(i) && wall_present_at_step.at(i)) {
                wall_absent_visualizer.change_beam_color_to_red();
                wall_present_visualizer.change_beam_color_to_red();
            } else {
                wall_absent_visualizer.reset_beam_color();
                wall_present_visualizer.reset_beam_color();
            }
            wall_absent_visualizer.draw_ir_3_sensor_beam(mouse, open_maze_distance);
            wall_present_visualizer.draw_ir_3_sensor_beam(mouse, closed_maze_distance);
        }

        mouse.translate(0.0, open_maze.cell_size / cfg.env_cfg.total_steps);
    }

    if (visualizer_enabled) {
        wall_absent_visualizer.save_to_image_file(TEST_OUTPUT_DIRECTORY + "/"
                                                  + TEST_OUTPUT_SUBDIRECTORY + "/"
                                                  + config_to_string(cfg) + "-wa.png");
        wall_present_visualizer.save_to_image_file(TEST_OUTPUT_DIRECTORY + "/"
                                                   + TEST_OUTPUT_SUBDIRECTORY + "/"
                                                   + config_to_string(cfg) + "-wp.png");
    }

    return Result{
        std::move(wall_absent_at_step),
        std::move(wall_present_at_step),
    };
}

WindowResult find_best_window(const std::vector<double>& rates, double window_fraction)
{
    const size_t n{rates.size()};
    if ((n == 0) || (window_fraction <= 0.0)) {
        return {};
    }

    const size_t window_size{
        std::max<size_t>(1, static_cast<size_t>(std::ceil(window_fraction * n)))};

    std::deque<size_t> dq; /* stores indices, monotonic increasing (for min) */

    double best_rate{-1.0};
    size_t best_start{0};

    for (size_t i{0}; i < n; ++i) {
        /* maintain increasing deque (front = minimum) */
        while (!dq.empty() && rates[dq.back()] >= rates[i]) {
            dq.pop_back();
        }
        dq.push_back(i);

        /* remove out-of-window indices */
        if (dq.front() + window_size <= i) {
            dq.pop_front();
        }

        /* evaluate window once fully formed */
        if ((i + 1) >= window_size) {
            const size_t start{i + 1 - window_size};
            const double window_min{rates[dq.front()]};

            if (window_min > best_rate) {
                best_rate = window_min;
                best_start = start;
            }
        }
    }

    WindowResult result;
    result.rate = (best_rate < 0.0) ? 0.0 : best_rate;
    result.start_fraction = static_cast<double>(best_start) / n;

    return result;
}

} /* side_wall_detection namespace */

/*----------------------------------------------------------------------------*/
/*                             Private Definitions                            */
/*----------------------------------------------------------------------------*/
namespace
{

using namespace side_wall_detection;

void prepare_mock_for_side_wall_detection(const Config& cfg, const maze::Maze& maze,
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
