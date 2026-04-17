/*-------------------------------- FILE INFO ---------------------------------*/
/* Filename           : rotation.cpp                                         */
/*                                                                            */
/* Implementation for micromouse rotation simulation and associated config    */
/* and results analysis helpers                                               */
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

}

#include <algorithm>
#include <cmath>
#include <cstdint>
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
#include "rotation.hpp"

/*----------------------------------------------------------------------------*/
/*                            Private Declarations                            */
/*----------------------------------------------------------------------------*/
namespace
{

using namespace rotation;

void prepare_mock_for_rotation(const Config& cfg, const maze::Maze& maze, mouse::Mouse& mouse);
mouse_delta update_mock_by_dt(const Config& cfg, mouse::Mouse& mouse);

} /* unnamed namespace */

/*----------------------------------------------------------------------------*/
/*                               Private Globals                              */
/*----------------------------------------------------------------------------*/
extern "C"
{

extern double ENCODER_TICKS_PER_ROTATION_ANGLE_RADIANS;

}

namespace
{

const std::string TEST_OUTPUT_DIRECTORY{"rotation-visualizer"};
std::string TEST_OUTPUT_SUBDIRECTORY{""};
bool visualizer_enabled{false};
visualizer::Visualizer rotation_visualizer;

} /* unnamed namespace */

/*----------------------------------------------------------------------------*/
/*                             Public Definitions                             */
/*----------------------------------------------------------------------------*/
namespace rotation
{

ControlConfig decode_control(const std::vector<double>& x)
{
    ControlConfig c{};
    size_t i{0};

    c.motor_speed = static_cast<uint8_t>(x.at(i++));
    c.kp_velocity = static_cast<int32_t>(x.at(i++));
    c.kd_velocity = static_cast<int32_t>(x.at(i++));
    c.kp_angle = static_cast<int32_t>(x.at(i++));
    c.kd_angle = static_cast<int32_t>(x.at(i++));
    c.pid_scale = static_cast<int32_t>(x.at(i++));

    return c;
}

std::vector<double> encode_control(const ControlConfig& cfg)
{
    return {static_cast<double>(cfg.motor_speed),
            static_cast<double>(cfg.kp_velocity),
            static_cast<double>(cfg.kd_velocity),
            static_cast<double>(cfg.kp_angle),
            static_cast<double>(cfg.kd_angle),
            static_cast<double>(cfg.pid_scale)};
}

std::pair<std::vector<double>, std::vector<double>> get_control_bounds(void)
{
    ControlConfig lower_bounds;
    lower_bounds.motor_speed = 140;
    lower_bounds.kp_velocity = 0;
    lower_bounds.kd_velocity = 0;
    lower_bounds.kp_angle = 0;
    lower_bounds.kd_angle = 0;
    lower_bounds.pid_scale = 16;

    ControlConfig upper_bounds;
    upper_bounds.motor_speed = 255;
    upper_bounds.kp_velocity = 2000;
    upper_bounds.kd_velocity = 2000;
    upper_bounds.kp_angle = 2000;
    upper_bounds.kd_angle = 2000;
    upper_bounds.pid_scale = 512;

    return {encode_control(lower_bounds), encode_control(upper_bounds)};
}

EnvironmentConfig generate_random_environment(void)
{
    static thread_local std::mt19937 rng(std::random_device{}());

    auto uniform = [&](double a, double b) {
        return std::uniform_real_distribution<double>(a, b)(rng);
    };

    EnvironmentConfig e;
    e.dt = uniform(0.005, 0.02);
    e.motor_speed_scale = uniform(0.9, 1.1);
    e.motor1_variance = uniform(-0.2, 0.2);
    e.motor2_variance = uniform(-0.2, 0.2);
    e.slip_factor = uniform(0.9, 1.1);
    e.wheel_circumference_scale = uniform(0.9, 1.1);
    e.wheel_base_scale = uniform(0.9, 1.1);
    e.rotation_angle = uniform(M_PI / 4, M_PI / 2);

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

    oss << simulation_common::double_to_filename(cfg.env_cfg.dt) << "-"
        << simulation_common::double_to_filename(cfg.env_cfg.motor_speed_scale) << "-"
        << simulation_common::double_to_filename(cfg.env_cfg.motor1_variance) << "-"
        << simulation_common::double_to_filename(cfg.env_cfg.motor2_variance) << "-"
        << simulation_common::double_to_filename(cfg.env_cfg.slip_factor) << "-"
        << simulation_common::double_to_filename(cfg.env_cfg.wheel_circumference_scale) << "-"
        << simulation_common::double_to_filename(cfg.env_cfg.wheel_base_scale) << "-"
        << static_cast<int>(cfg.ctrl_cfg.motor_speed) << "-"
        << simulation_common::double_to_filename(static_cast<double>(cfg.ctrl_cfg.kp_velocity)) << "-"
        << simulation_common::double_to_filename(static_cast<double>(cfg.ctrl_cfg.kd_velocity)) << "-"
        << simulation_common::double_to_filename(static_cast<double>(cfg.ctrl_cfg.kp_angle)) << "-"
        << simulation_common::double_to_filename(static_cast<double>(cfg.ctrl_cfg.kd_angle)) << "-"
        << simulation_common::double_to_filename(static_cast<double>(cfg.ctrl_cfg.pid_scale));

    return oss.str();
}

Result run_simulation(const Config& cfg)
{
    if (visualizer_enabled) {
        std::filesystem::create_directories(TEST_OUTPUT_DIRECTORY + "/" + TEST_OUTPUT_SUBDIRECTORY);
    }
    std::vector<std::string> ascii{
        "+-+",
        "|S|",
        "+-+"
    };
    maze::Maze maze{maze::build_maze_from_ascii(ascii, 0.0)};

    mouse::Mouse mouse;
    prepare_mock_for_rotation(cfg, maze, mouse);

    if (visualizer_enabled) {
        rotation_visualizer.draw_maze(100.0f, maze);
        rotation_visualizer.change_mouse_color_to_green();
        rotation_visualizer.draw_mouse_on_maze(mouse);
        rotation_visualizer.reset_mouse_color();
    }

    int target_angle_sign{0};
    if (cfg.env_cfg.rotation_angle > 0) {
        target_angle_sign = 1;
        set_wheel_motor_1_direction_backward();
        set_wheel_motor_2_direction_forward();
    } else {
        target_angle_sign = -1;
        set_wheel_motor_1_direction_forward();
        set_wheel_motor_2_direction_backward();
    }

    double total_translation{0.0};
    double total_angle_rotation{0.0};
    double total_time{0.0};
    bool collision{false};
    bool timeout{false};
    constexpr int MAX_STEPS{10000};
    int steps{0};

    int32_t prev_enc1{0};
    int32_t prev_enc2{0};
    int32_t prev_vel_error{0};
    int32_t prev_ang_error{0};
    double target_angle_absolute_value{std::abs(cfg.env_cfg.rotation_angle)};
    double raw_target{
        std::abs(ENCODER_TICKS_PER_ROTATION_ANGLE_RADIANS * target_angle_absolute_value)};
    int32_t target_ticks{static_cast<int32_t>(raw_target)};

    while (((std::abs(get_encoder_1_ticks()) + std::abs(get_encoder_2_ticks())) / 2)
           < target_ticks) {
        int32_t enc1{std::abs(get_encoder_1_ticks())};
        int32_t enc2{std::abs(get_encoder_2_ticks())};

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

        int64_t p_term_vel{static_cast<int64_t>(cfg.ctrl_cfg.kp_velocity) * vel_error};
        int64_t d_term_vel{static_cast<int64_t>(cfg.ctrl_cfg.kd_velocity) * vel_derivative};
        int64_t p_term_ang{static_cast<int64_t>(cfg.ctrl_cfg.kp_angle) * ang_error};
        int64_t d_term_ang{static_cast<int64_t>(cfg.ctrl_cfg.kd_angle) * ang_derivative};
        int64_t control64{p_term_vel + d_term_vel + p_term_ang + d_term_ang};
        int32_t control{static_cast<int32_t>(control64 / cfg.ctrl_cfg.pid_scale)};

        int32_t base_speed{static_cast<int32_t>(cfg.ctrl_cfg.motor_speed)};
        int32_t adjusted_speed_1{0};
        int32_t adjusted_speed_2{0};
        if (target_angle_sign > 0) {
            adjusted_speed_1 = base_speed + control;
            adjusted_speed_2 = base_speed - control;
        } else {
            adjusted_speed_1 = base_speed - control;
            adjusted_speed_2 = base_speed + control;
        }

        adjusted_speed_1 = std::clamp(adjusted_speed_1, 140, 255);
        adjusted_speed_2 = std::clamp(adjusted_speed_2, 140, 255);
        set_wheel_motor_1_speed(static_cast<uint8_t>(adjusted_speed_1));
        set_wheel_motor_2_speed(static_cast<uint8_t>(adjusted_speed_2));

        auto delta{update_mock_by_dt(cfg, mouse)};
        total_translation += sqrt((delta.dx * delta.dx) + (delta.dy * delta.dy));
        total_angle_rotation += delta.dtheta_rad;
        total_time += cfg.env_cfg.dt;

        if (visualizer_enabled) {
            rotation_visualizer.draw_mouse_on_maze(mouse);
        }

        if (maze::does_hitbox_collide_with_maze(maze, mouse.hitbox)) {
            collision = true;
            break;
        }

        steps++;
        if (steps > MAX_STEPS) {
            timeout = true;
            break;
        }
    }

    if (visualizer_enabled) {
        rotation_visualizer.change_mouse_color_to_blue();
        rotation_visualizer.draw_mouse_on_maze(mouse);
        rotation_visualizer.save_to_image_file(TEST_OUTPUT_DIRECTORY + "/"
                                               + TEST_OUTPUT_SUBDIRECTORY + "/"
                                               + config_to_string(cfg) + ".png");
    }

    return Result{
        total_time,
        std::abs(target_angle_absolute_value - std::abs(total_angle_rotation)),
        total_translation,
        collision,
        timeout
    };
}

} /* rotation namespace */

/*----------------------------------------------------------------------------*/
/*                             Private Definitions                            */
/*----------------------------------------------------------------------------*/
namespace
{

using namespace rotation;

void prepare_mock_for_rotation(const Config& cfg, const maze::Maze& maze, mouse::Mouse& mouse)
{
    reset_mock_device_drivers();
    set_motor_speed_scale(cfg.env_cfg.motor_speed_scale);
    set_motor_1_variance(cfg.env_cfg.motor1_variance);
    set_motor_2_variance(cfg.env_cfg.motor2_variance);
    set_motor_slip_factor(cfg.env_cfg.slip_factor);
    set_wheel_circumference_scale(cfg.env_cfg.wheel_circumference_scale);
    set_wheel_base_scale(cfg.env_cfg.wheel_base_scale);

    mouse.translate(maze.mouse_start.x, maze.mouse_start.y);
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

} /* unnamed namespace */
