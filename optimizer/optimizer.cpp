/*-------------------------------- FILE INFO ---------------------------------*/
/* Filename           : optimizer.cpp                                         */
/*                                                                            */
/* Implementation for micromouse simulations and optimal parameter generation */
/*                                                                            */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*                               Include Files                                */
/*----------------------------------------------------------------------------*/
extern "C"
{

#include <stdint.h>
#include <math.h>
#include "mock_device_drivers.h"
#include "wheel_motor.h"
#include "magnetic_encoder.h"

}

#include <cmath>
#include <cstdint>
#include <vector>
#include <string>
#include <optional>
#include <functional>
#include "point.hpp"
#include "ray.hpp"
#include "rectangular_hitbox.hpp"
#include "mouse.hpp"
#include "maze.hpp"
#include "optimizer.hpp"

/*----------------------------------------------------------------------------*/
/*                            Private Declarations                            */
/*----------------------------------------------------------------------------*/
namespace
{

using namespace optimizer;

RotationConfig build_rotation_config(const std::vector<double>& v);
RotationResult run_rotation_simulation(const maze::Maze& maze, const RotationConfig& cfg, double target_angle);

} /* unnamed namespace */

/*----------------------------------------------------------------------------*/
/*                               Private Globals                              */
/*----------------------------------------------------------------------------*/
extern "C"
{

extern double ENCODER_TICKS_PER_MILLIMETER;
extern double ENCODER_TICKS_PER_ROTATION_ANGLE_RADIANS;

}

/*----------------------------------------------------------------------------*/
/*                             Public Definitions                             */
/*----------------------------------------------------------------------------*/
namespace optimizer
{

SweepCursor::SweepCursor(const std::vector<SweepParam>& params)
    : params_(params), indices_(params.size(), 0)
{
    /* no additional logic */
}

bool SweepCursor::next()
{
    for (int i {(int)indices_.size() - 1}; i >= 0; i--) {
        indices_[i]++;

        if (indices_[i] < params_[i].steps)
            return true;

        indices_[i] = 0;
    }

    return false;
}

std::vector<double> SweepCursor::values() const
{
    std::vector<double> vals;
    vals.reserve(params_.size());

    for (size_t i {0}; i < params_.size(); i++) {
        const auto& p {params_[i]};

        double t = (p.steps == 1)
            ? 0.0
            : (double)indices_[i] / (p.steps - 1);

        vals.push_back(p.min + t * (p.max - p.min));
    }

    return vals;
}

RotationConfig build_rotation_config(const std::vector<double>& v)
{
    RotationConfig cfg{};

    int i {0};

    cfg.motor_speed = v[i++];
    cfg.motor_speed_scale = v[i++];
    cfg.dt = v[i++];

    cfg.motor1_variance = v[i++];
    cfg.motor2_variance = v[i++];
    cfg.slip_factor = v[i++];
    cfg.wheel_circumference_scale = v[i++];
    cfg.wheel_base_scale = v[i++];

    return cfg;
}

RotationResult run_rotation_simulation(const maze::Maze& maze,
        const RotationConfig& cfg, double target_angle)
{
    reset_mock_device_drivers();

    set_motor_speed_scale(cfg.motor_speed_scale);
    set_motor_1_variance(cfg.motor1_variance);
    set_motor_2_variance(cfg.motor2_variance);
    set_motor_slip_factor(cfg.slip_factor);
    set_wheel_circumference_scale(cfg.wheel_circumference_scale);
    set_wheel_base_scale(cfg.wheel_base_scale);

    mouse::Mouse mouse;
    mouse.translate(maze.mouse_start.x, maze.mouse_start.y);

    double total_translation {0.0};
    double time {0.0};
    bool collision {false};
    bool simulation_failed {false};

    set_wheel_motor_1_speed(cfg.motor_speed);
    set_wheel_motor_2_speed(cfg.motor_speed);
    if (target_angle > 0) {
        set_wheel_motor_1_direction_backward();
        set_wheel_motor_2_direction_forward();
    } else {
        set_wheel_motor_1_direction_forward();
        set_wheel_motor_2_direction_backward();
    }
    int target_encoder_count {static_cast<int>(std::abs(ENCODER_TICKS_PER_ROTATION_ANGLE_RADIANS * target_angle))};

    while ((std::abs(get_encoder_1_ticks()) < target_encoder_count)
            && (std::abs(get_encoder_2_ticks()) < target_encoder_count)) {
        /* update virtual mouse */
        auto delta {compute_mouse_delta(mouse.hitbox.angle_rad, cfg.dt)};
        int32_t new_encoder_1_ticks {compute_new_encoder_1_ticks(cfg.dt)};
        int32_t new_encoder_2_ticks {compute_new_encoder_2_ticks(cfg.dt)};
        if ((new_encoder_1_ticks == 0) || (new_encoder_2_ticks == 0)) {
            simulation_failed = true;
            break;
        }
        set_encoder_1_ticks(new_encoder_1_ticks);
        set_encoder_2_ticks(new_encoder_2_ticks);
        mouse.translate(delta.dx, delta.dy);
        mouse.rotate(delta.dtheta_rad);

        total_translation += sqrt(delta.dx * delta.dx + delta.dy * delta.dy);
        time += cfg.dt;

        auto rc {maze::get_cell_from_point(maze, mouse.hitbox.center)};
        if (rc) {
            auto [r, c] {*rc};
            if (maze::does_hitbox_collide_in_vicinity(maze, mouse.hitbox, r, c)) {
                collision = true;
                break;
            }
        }
    }

    return RotationResult{
        time,
        std::abs(target_angle - mouse.hitbox.angle_rad),
        total_translation,
        collision,
        simulation_failed
    };
}

} /* optimizer namespace */


/*----------------------------------------------------------------------------*/
/*                             Private Definitions                            */
/*----------------------------------------------------------------------------*/
/* none */