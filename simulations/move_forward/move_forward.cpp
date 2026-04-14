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
#include <cstdint>
#include <map>
#include <string>
#include <vector>
#include "point.hpp"
#include "ray.hpp"
#include "rectangular_hitbox.hpp"
#include "mouse.hpp"
#include "maze.hpp"
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

/*----------------------------------------------------------------------------*/
/*                             Public Definitions                             */
/*----------------------------------------------------------------------------*/
namespace move_forward
{

bool ConfigSweeper::next(void)
{
    if (!initialized_) {
        sweeper.init_sizes({
            dt.size(),
            motor_speed_scale.size(),
            motor1_variance.size(),
            motor2_variance.size(),
            slip_factor.size(),
            wheel_circumference_scale.size(),
            wheel_base_scale.size(),
            maze_size_scale.size(),
            ir_reading_scale.size(),
            mouse_angle.size(),
            horizontal_position_variance.size(),
            vertical_position_variance.size(),
            single_wall_target.size(),
            motor_speed.size(),
            kp.size(),
            kd.size(),
            pid_shift.size(),
            kp_ir.size(),
            kd_ir.size()
        });

        initialized_ = true;
    }
    return sweeper.next();
}

Config ConfigSweeper::value(void) const
{
    const auto& idx{sweeper.get_indices()};
    int i{0};

    Config cfg{};

    cfg.dt = dt.at(idx.at(i++));
    cfg.motor_speed_scale = motor_speed_scale.at(idx.at(i++));
    cfg.motor1_variance = motor1_variance.at(idx.at(i++));
    cfg.motor2_variance = motor2_variance.at(idx.at(i++));
    cfg.slip_factor = slip_factor.at(idx.at(i++));
    cfg.wheel_circumference_scale = wheel_circumference_scale.at(idx.at(i++));
    cfg.wheel_base_scale = wheel_base_scale.at(idx.at(i++));
    cfg.maze_size_scale = maze_size_scale.at(idx.at(i++));
    cfg.ir_reading_scale = ir_reading_scale.at(idx.at(i++));
    cfg.mouse_angle = mouse_angle.at(idx.at(i++));
    cfg.horizontal_position_variance = horizontal_position_variance.at(idx.at(i++));
    cfg.vertical_position_variance = vertical_position_variance.at(idx.at(i++));
    cfg.single_wall_target = single_wall_target.at(idx.at(i++));
    cfg.motor_speed = motor_speed.at(idx.at(i++));
    cfg.kp = kp.at(idx.at(i++));
    cfg.kd = kd.at(idx.at(i++));
    cfg.pid_shift = pid_shift.at(idx.at(i++));
    cfg.kp_ir = kp_ir.at(idx.at(i++));
    cfg.kd_ir = kd_ir.at(idx.at(i++));

    return cfg;
}

SingleCaseResult run_single_simulation(const Config& cfg, const maze::Maze& maze,
                                       enum wall_mode mode)
{
    mouse::Mouse mouse;
    prepare_mock_for_move_forward(cfg, maze, mouse);
    const double INITIAL_MOUSE_VERTICAL_POSITION{mouse.hitbox.center.y};

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

            int32_t ir2{static_cast<int32_t>(
                scale_and_clamp_ir_sensor_reading(read_ir_2_sensor(), cfg.ir_reading_scale))};
            int32_t ir3{static_cast<int32_t>(
                scale_and_clamp_ir_sensor_reading(read_ir_3_sensor(), cfg.ir_reading_scale))};

            const int32_t TARGET_IR_READING{static_cast<int32_t>(cfg.single_wall_target)};

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

        int64_t enc_control{(static_cast<int64_t>(cfg.kp) * encoder_error)
                            + (static_cast<int64_t>(cfg.kd) * enc_derivative)};

        /* IR PD */
        int32_t ir_derivative{ir_error - prev_ir_error};
        prev_ir_error = ir_error;

        int64_t ir_control{(static_cast<int64_t>(cfg.kp_ir) * ir_error)
                           + (static_cast<int64_t>(cfg.kd_ir) * ir_derivative)};

        /* combined feedback control */
        int64_t control64{enc_control + ir_control};
        int32_t control{(control64 >= 0) ? static_cast<int32_t>(control64 >> cfg.pid_shift)
                                         : -(static_cast<int32_t>((-control64) >> cfg.pid_shift))};
        int32_t base{cfg.motor_speed};
        int32_t speed1{std::clamp(base + control, 0, 255)};
        int32_t speed2{std::clamp(base - control, 0, 255)};

        set_wheel_motor_1_speed((uint8_t)speed1);
        set_wheel_motor_2_speed((uint8_t)speed2);

        auto delta{update_mock_by_dt(cfg, mouse)};
        total_horizontal_translation += std::abs(delta.dx);
        total_angle_error += std::abs(delta.dtheta_rad);
        total_time += cfg.dt;

        if (maze::does_hitbox_collide_with_maze(maze, mouse.hitbox)) {
            collision = true;
            break;
        }

        if (++steps > MAX_STEPS) {
            timeout = true;
            break;
        }
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
        ascii_no_walls, maze::OFFICIAL_POST_SIZE * (cfg.maze_size_scale - 1))};
    maze::Maze maze_left{maze::build_maze_from_ascii(
        ascii_left_wall, maze::OFFICIAL_POST_SIZE * (cfg.maze_size_scale - 1))};
    maze::Maze maze_both{maze::build_maze_from_ascii(
        ascii_both_walls, maze::OFFICIAL_POST_SIZE * (cfg.maze_size_scale - 1))};

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
    set_motor_speed_scale(cfg.motor_speed_scale);
    set_motor_1_variance(cfg.motor1_variance);
    set_motor_2_variance(cfg.motor2_variance);
    set_motor_slip_factor(cfg.slip_factor);
    set_wheel_circumference_scale(cfg.wheel_circumference_scale);
    set_wheel_base_scale(cfg.wheel_base_scale);

    double max_horizontal_offset{(maze::OFFICIAL_WALL_LENGTH_SIZE - mouse.hitbox.horizontal_size)
                                 / 2};
    double max_vertical_offset{(maze::OFFICIAL_WALL_LENGTH_SIZE - mouse.hitbox.vertical_size) / 2};

    mouse.rotate(cfg.mouse_angle);
    mouse.translate(maze.mouse_start.x + (max_horizontal_offset * cfg.horizontal_position_variance),
                    maze.mouse_start.y + (max_vertical_offset * cfg.vertical_position_variance));
}

mouse_delta update_mock_by_dt(const Config& cfg, mouse::Mouse& mouse)
{
    mouse_delta delta{compute_mouse_delta(mouse.hitbox.angle_rad, cfg.dt)};
    update_encoder_1_ticks(cfg.dt);
    update_encoder_2_ticks(cfg.dt);
    mouse.translate(delta.dx, delta.dy);
    mouse.rotate(delta.dtheta_rad);

    return delta;
}

} /* unnamed namespace*/
