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
#include <algorithm>
#include "point.hpp"
#include "ray.hpp"
#include "rectangular_hitbox.hpp"
#include "mouse.hpp"
#include "maze.hpp"
#include "optimizer.hpp"
#include "rotation.hpp"

/*----------------------------------------------------------------------------*/
/*                            Private Declarations                            */
/*----------------------------------------------------------------------------*/
/* none */

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
namespace rotation
{

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

RotationResult run_rotation_simulation(const maze::Maze& maze, const RotationConfig& cfg, double target_angle)
{
    /* prepare mouse for rotation */
    reset_mock_device_drivers();
    set_motor_speed_scale(cfg.motor_speed_scale);
    set_motor_1_variance(cfg.motor1_variance);
    set_motor_2_variance(cfg.motor2_variance);
    set_motor_slip_factor(cfg.slip_factor);
    set_wheel_circumference_scale(cfg.wheel_circumference_scale);
    set_wheel_base_scale(cfg.wheel_base_scale);
    set_wheel_motor_1_speed(cfg.motor_speed);
    set_wheel_motor_2_speed(cfg.motor_speed);
    if (target_angle > 0) {
        set_wheel_motor_1_direction_backward();
        set_wheel_motor_2_direction_forward();
    } else {
        set_wheel_motor_1_direction_forward();
        set_wheel_motor_2_direction_backward();
    }

    mouse::Mouse mouse;
    mouse.translate(maze.mouse_start.x, maze.mouse_start.y);

    double total_translation {0.0};
    double time {0.0};
    bool collision {false};
    bool simulation_failed {false};

    int target_encoder_count {static_cast<int>(std::abs(ENCODER_TICKS_PER_ROTATION_ANGLE_RADIANS * target_angle))};
    bool done_rotating {false};

    while (!done_rotating) {
        done_rotating = (std::abs(get_encoder_1_ticks()) >= target_encoder_count)
            && (std::abs(get_encoder_2_ticks()) >= target_encoder_count);

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

RotationAnalysisSummary analyze_rotation_results(const std::vector<std::pair<std::vector<double>,
        RotationResult>>& trials)
{
    RotationAnalysisSummary summary{};

    auto translations {optimizer::extract_metric(trials,
        [](const auto& t) { return t.second.total_translation; })};

    auto angle_errors {optimizer::extract_metric(trials,
        [](const auto& t) { return t.second.final_angle_error; })};

    summary.failure_rate = optimizer::compute_rate(trials,
        [](const auto& t) { return t.second.simulation_failed; });

    summary.collision_rate = optimizer::compute_rate(trials,
        [](const auto& t) { return t.second.collision; });

    summary.translation_stats = optimizer::compute_stats(translations);
    summary.angle_error_stats = optimizer::compute_stats(angle_errors);

    return summary;
}

std::vector<RotationParamImpact> analyze_rotation_parameter_impact(const std::vector<optimizer::SweepParam>& params,
        const std::vector<std::pair<std::vector<double>, RotationResult>>& trials)
{
    std::vector<RotationParamImpact> impacts;

    for (size_t i {0}; i < params.size(); i++) {
        RotationParamImpact impact{};
        impact.name = params[i].name;

        auto x {optimizer::extract_metric(trials,
            [i](const auto& t) { return t.first[i]; })};

        auto translation {optimizer::extract_metric(trials,
            [](const auto& t) { return t.second.total_translation; })};

        auto angle {optimizer::extract_metric(trials,
            [](const auto& t) { return t.second.final_angle_error; })};

        impact.correlation_translation = optimizer::compute_correlation(x, translation);
        impact.correlation_angle_error = optimizer::compute_correlation(x, angle);

        auto [fail_low, fail_high] {optimizer::compute_split_rate(
            trials,
            [i](const auto& t) { return t.first[i]; },
            [](const auto& t) { return t.second.simulation_failed; })};

        auto [coll_low, coll_high] {optimizer::compute_split_rate(
            trials,
            [i](const auto& t) { return t.first[i]; },
            [](const auto& t) { return t.second.collision; })};

        impact.failure_rate_low = fail_low;
        impact.failure_rate_high = fail_high;
        impact.collision_rate_low = coll_low;
        impact.collision_rate_high = coll_high;

        impacts.push_back(impact);
    }

    return impacts;
}

} /* rotation namespace */

/*----------------------------------------------------------------------------*/
/*                             Private Definitions                            */
/*----------------------------------------------------------------------------*/
/* none */
