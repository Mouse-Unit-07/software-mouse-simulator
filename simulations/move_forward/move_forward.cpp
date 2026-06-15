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
Result run_single_simulation(const Config& cfg, const maze::Maze& maze, enum WallMode mode);

} /* unnamed namespace*/

/*----------------------------------------------------------------------------*/
/*                               Private Globals                              */
/*----------------------------------------------------------------------------*/
extern "C"
{

extern double ENCODER_TICKS_PER_MILLIMETER;

}

namespace
{

constexpr double FLOAT_TOLERANCE{1e-6};
std::string TEST_OUTPUT_DIRECTORY{"visualizer"};
std::string TEST_OUTPUT_SUBDIRECTORY{""};
bool visualizer_enabled{false};
visualizer::Visualizer move_forward_visualizer;
ControlConfig ctr_lower_bounds{};
ControlConfig ctr_upper_bounds{};
EnvironmentConfig env_lower_bounds{};
EnvironmentConfig env_upper_bounds{};

} /* unnamed namespace*/

/*----------------------------------------------------------------------------*/
/*                             Public Definitions                             */
/*----------------------------------------------------------------------------*/
namespace move_forward
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

    c.single_wall_target = static_cast<uint32_t>(x.at(i++));
    c.motor_speed = static_cast<uint8_t>(x.at(i++));
    c.kp_velocity = static_cast<int32_t>(x.at(i++));
    c.kd_velocity = static_cast<int32_t>(x.at(i++));
    c.kp_angle = static_cast<int32_t>(x.at(i++));
    c.kd_angle = static_cast<int32_t>(x.at(i++));
    c.pid_scale = static_cast<int32_t>(x.at(i++));
    c.kp_ir = static_cast<int32_t>(x.at(i++));
    c.kd_ir = static_cast<int32_t>(x.at(i++));

    return c;
}

std::vector<double> encode_control(const ControlConfig& cfg)
{
    return {static_cast<double>(cfg.single_wall_target),
            static_cast<double>(cfg.motor_speed),
            static_cast<double>(cfg.kp_velocity),
            static_cast<double>(cfg.kd_velocity),
            static_cast<double>(cfg.kp_angle),
            static_cast<double>(cfg.kd_angle),
            static_cast<double>(cfg.pid_scale),
            static_cast<double>(cfg.kp_ir),
            static_cast<double>(cfg.kd_ir)};
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

    EnvironmentConfig e{};
    e.dt =
        uniform(env_lower_bounds.dt, env_upper_bounds.dt);
    e.motor_speed_scale =
        uniform(env_lower_bounds.motor_speed_scale, env_upper_bounds.motor_speed_scale);
    e.motor1_variance =
        uniform(env_lower_bounds.motor1_variance, env_upper_bounds.motor1_variance);
    e.motor2_variance =
        uniform(env_lower_bounds.motor2_variance, env_upper_bounds.motor2_variance);
    e.slip_factor =
        uniform(env_lower_bounds.slip_factor, env_upper_bounds.slip_factor);
    e.wheel_circumference_scale =
        uniform(env_lower_bounds.wheel_circumference_scale, env_upper_bounds.wheel_circumference_scale);
    e.wheel_base_scale =
        uniform(env_lower_bounds.wheel_base_scale, env_upper_bounds.wheel_base_scale);
    e.maze_post_size_scale =
        uniform(env_lower_bounds.maze_post_size_scale, env_upper_bounds.maze_post_size_scale);
    e.maze_wall_size_scale =
        uniform(env_lower_bounds.maze_wall_size_scale, env_upper_bounds.maze_wall_size_scale);
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
        << simulation_common::double_to_filename(cfg.env_cfg.maze_post_size_scale) << "-"
        << simulation_common::double_to_filename(cfg.env_cfg.maze_wall_size_scale) << "-"
        << simulation_common::double_to_filename(cfg.env_cfg.ir_reading_scale) << "-"
        << simulation_common::double_to_filename(cfg.env_cfg.mouse_angle) << "-"
        << simulation_common::double_to_filename(cfg.env_cfg.horizontal_position_variance) << "-"
        << simulation_common::double_to_filename(cfg.env_cfg.vertical_position_variance) << "-"
        << static_cast<int>(cfg.ctrl_cfg.single_wall_target) << "-"
        << static_cast<int>(cfg.ctrl_cfg.motor_speed) << "-"
        << simulation_common::double_to_filename(static_cast<double>(cfg.ctrl_cfg.kp_velocity)) << "-"
        << simulation_common::double_to_filename(static_cast<double>(cfg.ctrl_cfg.kd_velocity)) << "-"
        << simulation_common::double_to_filename(static_cast<double>(cfg.ctrl_cfg.kp_angle)) << "-"
        << simulation_common::double_to_filename(static_cast<double>(cfg.ctrl_cfg.kd_angle)) << "-"
        << simulation_common::double_to_filename(static_cast<double>(cfg.ctrl_cfg.pid_scale)) << "-"
        << simulation_common::double_to_filename(static_cast<double>(cfg.ctrl_cfg.kp_ir)) << "-"
        << simulation_common::double_to_filename(static_cast<double>(cfg.ctrl_cfg.kd_ir));

    return oss.str();
}

std::string wall_mode_to_string(WallMode mode)
{
    switch (mode) {
        case WallMode::NO_WALLS:
            return "no-walls";
        case WallMode::LEFT_WALL_ONLY:
            return "left-wall";
        case WallMode::BOTH_WALLS:
            return "both-walls";
        default:
            return "unknown";
    }
}

Result run_simulation(const Config& cfg, enum WallMode mode)
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
    maze::Maze maze_none{maze::build_maze_from_ascii(ascii_no_walls,
                                                     cfg.env_cfg.maze_post_size_scale,
                                                     cfg.env_cfg.maze_wall_size_scale)};

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
    maze::Maze maze_left{maze::build_maze_from_ascii(ascii_left_wall,
                                                     cfg.env_cfg.maze_post_size_scale,
                                                     cfg.env_cfg.maze_wall_size_scale)};

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
    maze::Maze maze_both{maze::build_maze_from_ascii(ascii_both_walls,
                                                     cfg.env_cfg.maze_post_size_scale,
                                                     cfg.env_cfg.maze_wall_size_scale)};

    Result out;
    if (mode == WallMode::NO_WALLS) {
        out = run_single_simulation(cfg, maze_none, mode);
    } else if (mode == WallMode::LEFT_WALL_ONLY) {
        out = run_single_simulation(cfg, maze_left, mode);
    } else if (mode == WallMode::BOTH_WALLS) {
        out = run_single_simulation(cfg, maze_both, mode);
    }

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

    double max_horizontal_offset{(maze.wall_length_size - mouse.hitbox.horizontal_size) / 2};
    double max_vertical_offset{(maze.wall_length_size - mouse.hitbox.vertical_size) / 2};

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

Result run_single_simulation(const Config& cfg, const maze::Maze& maze, enum WallMode mode)
{
    if (visualizer_enabled) {
        std::filesystem::create_directories(TEST_OUTPUT_DIRECTORY + "/" + TEST_OUTPUT_SUBDIRECTORY);
    }

    mouse::Mouse mouse{};
    const double IDEAL_MOUSE_ANGLE{mouse.hitbox.angle_rad};
    prepare_mock_for_move_forward(cfg, maze, mouse);
    const double INITIAL_MOUSE_VERTICAL_POSITION{mouse.hitbox.center.y};

    if (visualizer_enabled) {
        move_forward_visualizer.draw_maze(100.0f, maze);
        move_forward_visualizer.change_mouse_color_to_green();
        move_forward_visualizer.draw_mouse_on_maze(mouse);
        move_forward_visualizer.reset_mouse_color();
    }

    set_wheel_motor_1_direction_forward();
    set_wheel_motor_2_direction_forward();

    clear_1_encoder_ticks();
    clear_2_encoder_ticks();

    constexpr int MAX_STEPS{20000};
    int steps{0};
    constexpr int MAZE_SQUARE_COUNT{2};

    double total_time{0.0};
    bool collision{false};
    bool timeout{false};

    int32_t prev_enc1{0};
    int32_t prev_enc2{0};
    int32_t prev_vel_error{0};
    int32_t prev_ang_error{0};
    int32_t prev_ir_error{0};

    const double TARGET_DISTANCE_MM{maze.cell_size * MAZE_SQUARE_COUNT};
    const int32_t TARGET_TICKS{
        static_cast<int32_t>(TARGET_DISTANCE_MM * ENCODER_TICKS_PER_MILLIMETER)};

    while (((std::abs(get_encoder_1_ticks()) + std::abs(get_encoder_2_ticks())) / 2)
           < TARGET_TICKS) {
        int32_t enc1{get_encoder_1_ticks()};
        int32_t enc2{get_encoder_2_ticks()};

        int32_t vel1{enc1 - prev_enc1};
        int32_t vel2{enc2 - prev_enc2};
        int32_t vel_error{vel2 - vel1};
        int32_t vel_derivative{vel_error - prev_vel_error};
        prev_vel_error = vel_error;

        int32_t ang_error{enc2 - enc1};
        int32_t ang_derivative{ang_error - prev_ang_error};
        prev_ang_error = ang_error;

        prev_enc1 = enc1;
        prev_enc2 = enc2;

        int32_t ir_error{0};

        if (mode != WallMode::NO_WALLS) {
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

            if (mode == WallMode::LEFT_WALL_ONLY) {
                ir_error = (TARGET_IR_READING - ir2);
            } else if (mode == WallMode::RIGHT_WALL_ONLY) {
                ir_error = -(TARGET_IR_READING - ir3);
            } else if (mode == WallMode::BOTH_WALLS) {
                ir_error = (ir3 - ir2);
            }
        }
        int32_t ir_derivative{ir_error - prev_ir_error};
        prev_ir_error = ir_error;

        /* encoder PD */
        int64_t p_term_vel{static_cast<int64_t>(cfg.ctrl_cfg.kp_velocity) * vel_error};
        int64_t d_term_vel{static_cast<int64_t>(cfg.ctrl_cfg.kd_velocity) * vel_derivative};
        int64_t p_term_ang{static_cast<int64_t>(cfg.ctrl_cfg.kp_angle) * ang_error};
        int64_t d_term_ang{static_cast<int64_t>(cfg.ctrl_cfg.kd_angle) * ang_derivative};
        int64_t enc_control{p_term_vel + d_term_vel + p_term_ang + d_term_ang};

        /* IR PD */
        int64_t p_term_ir{static_cast<int64_t>(cfg.ctrl_cfg.kp_ir) * ir_error};
        int64_t d_term_ir{static_cast<int64_t>(cfg.ctrl_cfg.kd_ir) * ir_derivative};
        int64_t ir_control{p_term_ir + d_term_ir};

        /* combined feedback control */
        int64_t control64{enc_control + ir_control};
        int32_t control{static_cast<int32_t>(control64 / cfg.ctrl_cfg.pid_scale)};
        int32_t base{cfg.ctrl_cfg.motor_speed};
        int32_t speed1{std::clamp(base + control, 55, 255)};
        int32_t speed2{std::clamp(base - control, 55, 255)};

        set_wheel_motor_1_speed(static_cast<uint8_t>(speed1));
        set_wheel_motor_2_speed(static_cast<uint8_t>(speed2));

        update_mock_by_dt(cfg, mouse);
        total_time += cfg.env_cfg.dt;

        if (visualizer_enabled) {
            move_forward_visualizer.draw_mouse_on_maze(mouse);
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
        move_forward_visualizer.change_mouse_color_to_blue();
        move_forward_visualizer.draw_mouse_on_maze(mouse);
        move_forward_visualizer.save_to_image_file(
            TEST_OUTPUT_DIRECTORY + "/" + TEST_OUTPUT_SUBDIRECTORY + "/" + config_to_string(cfg)
            + "-" + wall_mode_to_string(mode) + ".png");
    }

    double target_y{INITIAL_MOUSE_VERTICAL_POSITION + (maze.cell_size * MAZE_SQUARE_COUNT)};
    double final_vertical_translation{std::abs(target_y - mouse.hitbox.center.y)};
    double final_horizontal_translation{std::abs(maze.mouse_start.x - mouse.hitbox.center.x)};

    double final_angle_error{std::abs(IDEAL_MOUSE_ANGLE - std::abs(mouse.hitbox.angle_rad))};

    return Result{total_time,
                  final_angle_error,
                  final_horizontal_translation,
                  final_vertical_translation,
                  collision,
                  timeout};
}

} /* unnamed namespace*/
