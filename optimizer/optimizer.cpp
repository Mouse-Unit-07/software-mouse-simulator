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
#include <algorithm>
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

MetricStats compute_stats(const std::vector<double>& data);
double compute_correlation(const std::vector<double>& x, const std::vector<double>& y);

template <typename Trials, typename Fn>
std::vector<double> extract_metric(const Trials& trials, Fn fn);

template <typename Trials, typename Pred>
double compute_rate(const Trials& trials, Pred pred);

template <typename Trials, typename ValueFn, typename Pred>
std::pair<double, double> compute_split_rate(const Trials& trials, ValueFn value_fn, Pred pred);

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

        double t {(p.steps == 1) ? 0.0 : static_cast<double>(indices_[i]) / (p.steps - 1)};

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

    auto translations {extract_metric(trials,
        [](const auto& t) { return t.second.total_translation; })};

    auto angle_errors {extract_metric(trials,
        [](const auto& t) { return t.second.final_angle_error; })};

    summary.failure_rate = compute_rate(trials,
        [](const auto& t) { return t.second.simulation_failed; });

    summary.collision_rate = compute_rate(trials,
        [](const auto& t) { return t.second.collision; });

    summary.translation_stats = compute_stats(translations);
    summary.angle_error_stats = compute_stats(angle_errors);

    return summary;
}

std::vector<RotationParamImpact> analyze_rotation_parameter_impact(const std::vector<SweepParam>& params,
        const std::vector<std::pair<std::vector<double>, RotationResult>>& trials)
{
    std::vector<RotationParamImpact> impacts;

    for (size_t i {0}; i < params.size(); i++) {
        RotationParamImpact impact{};
        impact.name = params[i].name;

        auto x {extract_metric(trials,
            [i](const auto& t) { return t.first[i]; })};

        auto translation {extract_metric(trials,
            [](const auto& t) { return t.second.total_translation; })};

        auto angle {extract_metric(trials,
            [](const auto& t) { return t.second.final_angle_error; })};

        impact.correlation_translation = compute_correlation(x, translation);
        impact.correlation_angle_error = compute_correlation(x, angle);

        auto [fail_low, fail_high] {compute_split_rate(
            trials,
            [i](const auto& t) { return t.first[i]; },
            [](const auto& t) { return t.second.simulation_failed; })};

        auto [coll_low, coll_high] {compute_split_rate(
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

} /* optimizer namespace */


/*----------------------------------------------------------------------------*/
/*                             Private Definitions                            */
/*----------------------------------------------------------------------------*/
namespace
{

MetricStats compute_stats(const std::vector<double>& data)
{
    MetricStats s{};

    if (data.empty()) {
        return s;
    }

    double sum {0.0};
    s.min = data[0];
    s.max = data[0];

    for (double v : data) {
        sum += v;
        if (v < s.min) {
            s.min = v;
        }
        if (v > s.max) {
            s.max = v;
        }
    }

    s.mean = sum / data.size();

    double variance {0.0};
    for (double v : data) {
        double d {v - s.mean};
        variance += d * d;
    }

    s.stddev = sqrt(variance / data.size());
    return s;
}

double compute_correlation(const std::vector<double>& x, const std::vector<double>& y)
{
    if ((x.size() != y.size()) || x.empty()) {
        return 0.0;
    }

    double mean_x {0.0};
    double mean_y {0.0};

    for (size_t i = 0; i < x.size(); i++) {
        mean_x += x[i];
        mean_y += y[i];
    }

    mean_x /= x.size();
    mean_y /= y.size();

    double num {0.0};
    double den_x {0.0};
    double den_y {0.0};

    for (size_t i = 0; i < x.size(); i++) {
        double dx {x[i] - mean_x};
        double dy {y[i] - mean_y};

        num += dx * dy;
        den_x += dx * dx;
        den_y += dy * dy;
    }

    return num / sqrt(den_x * den_y + 1e-9);
}

template <typename Trials, typename Fn>
std::vector<double> extract_metric(const Trials& trials, Fn fn)
{
    std::vector<double> out;
    out.reserve(trials.size());

    for (const auto& t : trials) {
        out.push_back(fn(t));
    }

    return out;
}

template <typename Trials, typename Pred>
double compute_rate(const Trials& trials, Pred pred)
{
    if (trials.empty()) {
        return 0.0;
    }

    int count {0};
    for (const auto& t : trials) {
        if (pred(t)) {
            count++;
        }
    }

    return static_cast<double>(count) / trials.size();
}

template <typename Trials, typename ValueFn, typename Pred>
std::pair<double, double> compute_split_rate(const Trials& trials, ValueFn value_fn, Pred pred)
{
    if (trials.empty()) {
        return {0.0, 0.0};
    }

    std::vector<double> values;
    values.reserve(trials.size());

    for (const auto& t : trials) {
        values.push_back(value_fn(t));
    }

    std::sort(values.begin(), values.end());
    double mid {values[values.size() / 2]};

    int low_total {0}, high_total {0};
    int low_count {0}, high_count {0};

    for (const auto& t : trials) {
        if (value_fn(t) < mid) {
            low_total++;
            if (pred(t)) {
                low_count++;
            }
        } else {
            high_total++;
            if (pred(t)) {
                high_count++;
            }
        }
    }

    double low_rate {(low_total > 0) ? static_cast<double>(low_count) / low_total : 0.0};
    double high_rate {(high_total > 0) ? static_cast<double>(high_count) / high_total : 0.0};

    return {low_rate, high_rate};
}

} /* unnamed namespace */