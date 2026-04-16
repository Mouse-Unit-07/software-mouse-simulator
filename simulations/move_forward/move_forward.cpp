/*-------------------------------- FILE INFO ---------------------------------*/
/* Filename           : move_forward.cpp                                      */
/*                                                                            */
/* Implementation for micromouse move_forward simulation and associated       */
/* config and results analysis helpers                                        */
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
#include "wheel_motor.h"
#include "magnetic_encoder.h"
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
#include "move_forward.hpp"

/*----------------------------------------------------------------------------*/
/*                            Private Declarations                            */
/*----------------------------------------------------------------------------*/
namespace
{

using namespace move_forward;

void prepare_mock_for_move_forward(const Config& cfg, const maze::Maze& maze, mouse::Mouse& mouse);
mouse_delta update_mock_by_dt(const Config& cfg, mouse::Mouse& mouse);

} /* unnamed namespace*/

/*----------------------------------------------------------------------------*/
/*                               Private Globals                              */
/*----------------------------------------------------------------------------*/
extern "C"
{

extern double ENCODER_TICKS_PER_MILLIMETER;

}

constexpr double FLOAT_TOLERANCE{1e-6};
std::string TEST_OUTPUT_DIRECTORY{"rotation-visualizer"};
std::string TEST_OUTPUT_SUBDIRECTORY{""};
bool visualizer_enabled{false};
visualizer::Visualizer rotation_visualizer;

/*----------------------------------------------------------------------------*/
/*                             Public Definitions                             */
/*----------------------------------------------------------------------------*/
namespace move_forward
{

ControlConfig decode_control(const std::vector<double>& x)
{
    ControlConfig c{};
    size_t i{0};

    c.single_wall_target = static_cast<uint32_t>(x.at(i++));
    c.motor_speed = static_cast<uint8_t>(x.at(i++));
    c.kp = static_cast<int32_t>(x.at(i++));
    c.kd = static_cast<int32_t>(x.at(i++));
    c.pid_shift = static_cast<int32_t>(x.at(i++));
    c.kp_ir = static_cast<int32_t>(x.at(i++));
    c.kd_ir = static_cast<int32_t>(x.at(i++));

    return c;
}

std::vector<double> encode_control(const ControlConfig& cfg)
{
    return {static_cast<double>(cfg.single_wall_target),
            static_cast<double>(cfg.motor_speed),
            static_cast<double>(cfg.kp),
            static_cast<double>(cfg.kd),
            static_cast<double>(cfg.pid_shift),
            static_cast<double>(cfg.kp_ir),
            static_cast<double>(cfg.kd_ir)};
}

std::pair<std::vector<double>, std::vector<double>> get_control_bounds(void)
{
    ControlConfig lower_bounds;
    lower_bounds.single_wall_target = 0;
    lower_bounds.motor_speed = 100;
    lower_bounds.kp = 0;
    lower_bounds.kd = 0;
    lower_bounds.pid_shift = 4;
    lower_bounds.kp_ir = 4;
    lower_bounds.kd_ir = 4;

    ControlConfig upper_bounds;
    upper_bounds.single_wall_target = 1024;
    upper_bounds.motor_speed = 255;
    upper_bounds.kp = 2000;
    upper_bounds.kd = 2000;
    upper_bounds.pid_shift = 8;
    upper_bounds.kp_ir = 2000;
    upper_bounds.kd_ir = 2000;

    return {encode_control(lower_bounds), encode_control(upper_bounds)};
}

EnvironmentConfig generate_random_environment(void)
{
    static thread_local std::mt19937 rng(std::random_device{}());

    auto uniform = [&](double a, double b) {
        return std::uniform_real_distribution<double>(a, b)(rng);
    };

    EnvironmentConfig e;
    e.dt = uniform(0.01, 0.1);
    e.motor_speed_scale = uniform(0.9, 1.1);
    e.motor1_variance = uniform(-0.2, 0.2);
    e.motor2_variance = uniform(-0.2, 0.2);
    e.slip_factor = uniform(0.9, 1.1);
    e.wheel_circumference_scale = uniform(0.9, 1.1);
    e.wheel_base_scale = uniform(0.9, 1.1);
    e.maze_size_scale = uniform(0.9, 1.1);
    e.ir_reading_scale = uniform(0.9, 1.1);
    e.mouse_angle = uniform(-(M_PI / 4), M_PI / 4);
    e.horizontal_position_variance = uniform(-0.5, 0.5);
    e.vertical_position_variance = uniform(-0.5, 0.5);

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
    TEST_OUTPUT_SUBDIRECTORY = "";
}

std::string config_to_string(const Config& cfg)
{
    std::ostringstream oss;

    oss << simulation_common::double_to_filename(cfg.env_cfg.dt) << "-"
        << simulation_common::double_to_filename(cfg.env_cfg.motor_speed_scale) << "-"
        << simulation_common::double_to_filename(cfg.env_cfg.motor1_variance) << "-"
        << simulation_common::double_to_filename(cfg.env_cfg.motor2_variance) << "-"
        << simulation_common::double_to_filename(cfg.env_cfg.slip_factor) << "-"
        << simulation_common::double_to_filename(cfg.env_cfg.wheel_circumference_scale) << "-"
        << simulation_common::double_to_filename(cfg.env_cfg.wheel_base_scale) << "-"
        << simulation_common::double_to_filename(cfg.env_cfg.maze_size_scale) << "-"
        << simulation_common::double_to_filename(cfg.env_cfg.ir_reading_scale) << "-"
        << simulation_common::double_to_filename(cfg.env_cfg.mouse_angle) << "-"
        << simulation_common::double_to_filename(cfg.env_cfg.horizontal_position_variance) << "-"
        << simulation_common::double_to_filename(cfg.env_cfg.vertical_position_variance) << "-"
        << static_cast<int>(cfg.ctrl_cfg.single_wall_target) << "-"
        << static_cast<int>(cfg.ctrl_cfg.motor_speed) << "-"
        << simulation_common::double_to_filename(static_cast<double>(cfg.ctrl_cfg.kp)) << "-"
        << simulation_common::double_to_filename(static_cast<double>(cfg.ctrl_cfg.kd)) << "-"
        << simulation_common::double_to_filename(static_cast<double>(cfg.ctrl_cfg.pid_shift)) << "-"
        << simulation_common::double_to_filename(static_cast<double>(cfg.ctrl_cfg.kp_ir)) << "-"
        << simulation_common::double_to_filename(static_cast<double>(cfg.ctrl_cfg.kd_ir));

    return oss.str();
}

SingleCaseResult run_single_simulation(const Config& cfg, const maze::Maze& maze,
                                       enum wall_mode mode)
{
    if (visualizer_enabled) {
        std::filesystem::create_directories(TEST_OUTPUT_DIRECTORY + "/" + TEST_OUTPUT_SUBDIRECTORY);
    }

    mouse::Mouse mouse;
    prepare_mock_for_move_forward(cfg, maze, mouse);
    const double INITIAL_MOUSE_VERTICAL_POSITION{mouse.hitbox.center.y};

    if (visualizer_enabled) {
        rotation_visualizer.draw_maze(100.0f, maze);
        rotation_visualizer.change_mouse_color_to_green();
        rotation_visualizer.draw_mouse_on_maze(mouse);
        rotation_visualizer.reset_mouse_color();
    }

    set_wheel_motor_1_direction_forward();
    set_wheel_motor_2_direction_forward();

    clear_1_encoder_ticks();
    clear_2_encoder_ticks();

    constexpr int MAX_STEPS{20000};
    int steps{0};
    constexpr int MAZE_SQUARE_COUNT{2};

    double total_time{0.0};
    double total_angle_error{0.0};
    double total_horizontal_translation{0.0};
    bool collision{false};
    bool timeout{false};

    int32_t prev_encoder_error{0};
    int32_t prev_ir_error{0};

    const double TARGET_DISTANCE_MM{maze.cell_size * MAZE_SQUARE_COUNT};
    const int32_t TARGET_TICKS{
        static_cast<int32_t>(TARGET_DISTANCE_MM * ENCODER_TICKS_PER_MILLIMETER)};

    while (true) {
        int32_t enc1{get_encoder_1_ticks()};
        int32_t enc2{get_encoder_2_ticks()};
        int32_t avg_ticks{(enc1 + enc2) / 2};

        if (avg_ticks >= TARGET_TICKS) {
            break;
        }

        int32_t encoder_error{enc2 - enc1};
        int32_t ir_error{0};

        if (mode != NO_WALLS) {
            double ir2_dist{maze::compute_ray_distance_in_open_space(maze, mouse.hitbox.center,
                                                                     mouse.ir_2_sensor)};
            double ir3_dist{maze::compute_ray_distance_in_open_space(maze, mouse.hitbox.center,
                                                                     mouse.ir_3_sensor)};

            update_ir_2_sensor_reading(ir2_dist);
            update_ir_3_sensor_reading(ir3_dist);

            int32_t ir2{static_cast<int32_t>(scale_and_clamp_ir_sensor_reading(
                read_ir_2_sensor(), cfg.env_cfg.ir_reading_scale))};
            int32_t ir3{static_cast<int32_t>(scale_and_clamp_ir_sensor_reading(
                read_ir_3_sensor(), cfg.env_cfg.ir_reading_scale))};

            const int32_t TARGET_IR_READING{static_cast<int32_t>(cfg.ctrl_cfg.single_wall_target)};

            if (mode == LEFT_WALL_ONLY) {
                ir_error = (TARGET_IR_READING - ir2);
            } else if (mode == RIGHT_WALL_ONLY) {
                ir_error = -(TARGET_IR_READING - ir3);
            } else if (mode == BOTH_WALLS) {
                ir_error = (ir3 - ir2);
            }
        }

        /* encoder PD */
        int32_t enc_derivative{encoder_error - prev_encoder_error};
        prev_encoder_error = encoder_error;

        int64_t enc_control{(static_cast<int64_t>(cfg.ctrl_cfg.kp) * encoder_error)
                            + (static_cast<int64_t>(cfg.ctrl_cfg.kd) * enc_derivative)};

        /* IR PD */
        int32_t ir_derivative{ir_error - prev_ir_error};
        prev_ir_error = ir_error;

        int64_t ir_control{(static_cast<int64_t>(cfg.ctrl_cfg.kp_ir) * ir_error)
                           + (static_cast<int64_t>(cfg.ctrl_cfg.kd_ir) * ir_derivative)};

        /* combined feedback control */
        int64_t control64{enc_control + ir_control};
        int32_t control{(control64 >= 0)
                            ? static_cast<int32_t>(control64 >> cfg.ctrl_cfg.pid_shift)
                            : -(static_cast<int32_t>((-control64) >> cfg.ctrl_cfg.pid_shift))};
        int32_t base{cfg.ctrl_cfg.motor_speed};
        int32_t speed1{std::clamp(base + control, 0, 255)};
        int32_t speed2{std::clamp(base - control, 0, 255)};

        set_wheel_motor_1_speed((uint8_t)speed1);
        set_wheel_motor_2_speed((uint8_t)speed2);

        auto delta{update_mock_by_dt(cfg, mouse)};
        total_horizontal_translation += std::abs(delta.dx);
        total_angle_error += std::abs(delta.dtheta_rad);
        total_time += cfg.env_cfg.dt;

        if (visualizer_enabled) {
            rotation_visualizer.draw_mouse_on_maze(mouse);
        }

        if (maze::does_hitbox_collide_with_maze(maze, mouse.hitbox)) {
            collision = true;
            break;
        }

        if (++steps > MAX_STEPS) {
            timeout = true;
            break;
        }
    }

    if (visualizer_enabled) {
        rotation_visualizer.change_mouse_color_to_blue();
        rotation_visualizer.draw_mouse_on_maze(mouse);
        rotation_visualizer.save_to_image_file(
            TEST_OUTPUT_DIRECTORY + "/" + TEST_OUTPUT_SUBDIRECTORY + "/" + config_to_string(cfg)
            + "-" + std::to_string(mode) + ".png");
    }

    double target_y{INITIAL_MOUSE_VERTICAL_POSITION + (maze.cell_size * MAZE_SQUARE_COUNT)};
    double final_vertical_translation{std::abs(target_y - mouse.hitbox.center.y)};

    return SingleCaseResult{
        total_time,
        total_angle_error,
        total_horizontal_translation,
        final_vertical_translation,
        collision,
        timeout
    };
}

Result run_simulation(const Config& cfg)
{
    std::vector<std::string> ascii_no_walls{
        "+-+",
        " S ",
        "   ",
        "   ",
        "   ",
        "   ",
        "   ",
        "   ",
        "   "
    };

    std::vector<std::string> ascii_left_wall{
        "+-+",
        " S|",
        "  +",
        "  |",
        "  +",
        "  |",
        "  +",
        "  |",
        "  +"
    };

    std::vector<std::string> ascii_both_walls{
        "+-+",
        "|S|",
        "+ +",
        "| |",
        "+ +",
        "| |",
        "+ +",
        "| |",
        "+ +"
    };

    maze::Maze maze_none{maze::build_maze_from_ascii(
        ascii_no_walls, maze::OFFICIAL_POST_SIZE * (cfg.env_cfg.maze_size_scale - 1))};
    maze::Maze maze_left{maze::build_maze_from_ascii(
        ascii_left_wall, maze::OFFICIAL_POST_SIZE * (cfg.env_cfg.maze_size_scale - 1))};
    maze::Maze maze_both{maze::build_maze_from_ascii(
        ascii_both_walls, maze::OFFICIAL_POST_SIZE * (cfg.env_cfg.maze_size_scale - 1))};

    Result out;

    out.no_wall = run_single_simulation(cfg, maze_none, NO_WALLS);
    out.one_wall = run_single_simulation(cfg, maze_left, LEFT_WALL_ONLY);
    out.two_wall = run_single_simulation(cfg, maze_both, BOTH_WALLS);

    return out;
}

} /* move_forward namespace */

/*----------------------------------------------------------------------------*/
/*                             Private Definitions                            */
/*----------------------------------------------------------------------------*/
namespace
{

using namespace move_forward;

void prepare_mock_for_move_forward(const Config& cfg, const maze::Maze& maze, mouse::Mouse& mouse)
{
    reset_mock_device_drivers();
    set_motor_speed_scale(cfg.env_cfg.motor_speed_scale);
    set_motor_1_variance(cfg.env_cfg.motor1_variance);
    set_motor_2_variance(cfg.env_cfg.motor2_variance);
    set_motor_slip_factor(cfg.env_cfg.slip_factor);
    set_wheel_circumference_scale(cfg.env_cfg.wheel_circumference_scale);
    set_wheel_base_scale(cfg.env_cfg.wheel_base_scale);

    double max_horizontal_offset{(maze::OFFICIAL_WALL_LENGTH_SIZE - mouse.hitbox.horizontal_size)
                                 / 2};
    double max_vertical_offset{(maze::OFFICIAL_WALL_LENGTH_SIZE - mouse.hitbox.vertical_size) / 2};

    mouse.rotate(cfg.env_cfg.mouse_angle);
    mouse.translate(
        maze.mouse_start.x + (max_horizontal_offset * cfg.env_cfg.horizontal_position_variance),
        maze.mouse_start.y + (max_vertical_offset * cfg.env_cfg.vertical_position_variance));
}

mouse_delta update_mock_by_dt(const Config& cfg, mouse::Mouse& mouse)
{
    mouse_delta delta{compute_mouse_delta(mouse.hitbox.angle_rad, cfg.env_cfg.dt)};
    update_encoder_1_ticks(cfg.env_cfg.dt);
    update_encoder_2_ticks(cfg.env_cfg.dt);
    mouse.translate(delta.dx, delta.dy);
    mouse.rotate(delta.dtheta_rad);

    return delta;
}

} /* unnamed namespace*/
