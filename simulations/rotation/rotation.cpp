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
#include <map>
#include <iostream>
#include <iomanip>
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

    cfg.kp = static_cast<int32_t>(v[i++]);
    cfg.kd = static_cast<int32_t>(v[i++]);
    cfg.pid_shift = static_cast<int32_t>(v[i++]);

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

    int32_t prev_error1 {0}, prev_error2 {0};

    const int32_t ERROR_EPSILON {5};
    const int MAX_STEPS {10000};

    int steps {0};

    double raw_target = std::abs(ENCODER_TICKS_PER_ROTATION_ANGLE_RADIANS * target_angle);
    if (raw_target > static_cast<double>(INT32_MAX)) {
        raw_target = INT32_MAX;
    }
    int32_t target_ticks {static_cast<int32_t>(raw_target)};
    bool done_rotating {false};

    while (!done_rotating) {
        int32_t enc1 {get_encoder_1_ticks()};
        int32_t enc2 {get_encoder_2_ticks()};
        enc1 = (enc1 == INT32_MIN) ? INT32_MAX : std::abs(enc1);
        enc2 = (enc2 == INT32_MIN) ? INT32_MAX : std::abs(enc2);
        
        int32_t error1 {target_ticks - enc1};
        int32_t error2 {target_ticks - enc2};
        done_rotating = (std::abs(error1) <= ERROR_EPSILON) && (std::abs(error2) <= ERROR_EPSILON);

        int32_t derivative1 {error1 - prev_error1};
        int32_t derivative2 {error2 - prev_error2};

        int64_t p_term1 {static_cast<int64_t>(cfg.kp) * error1};
        int64_t p_term2 {static_cast<int64_t>(cfg.kp) * error2};
        int64_t d_term1 {static_cast<int64_t>(cfg.kd) * derivative1};
        int64_t d_term2 {static_cast<int64_t>(cfg.kd) * derivative2};
        int64_t control64_1 {p_term1 + d_term1};
        int64_t control64_2 {p_term2 + d_term2};

        int32_t control_1, control_2;
        if (control64_1 >= 0) {
            control_1 = static_cast<int32_t>(control64_1 >> cfg.pid_shift);
        } else {
            control_1 = -static_cast<int32_t>((-control64_1) >> cfg.pid_shift);
        }
        if (control64_2 >= 0) {
            control_2 = static_cast<int32_t>(control64_2 >> cfg.pid_shift);
        } else {
            control_2 = -static_cast<int32_t>((-control64_2) >> cfg.pid_shift);
        }

        int32_t base_speed {static_cast<int32_t>(cfg.motor_speed)};
        int32_t adjusted_speed_1 {base_speed + control_1};
        int32_t adjusted_speed_2 {base_speed - control_2};
        adjusted_speed_1 = std::clamp(adjusted_speed_1, 0, 255);
        adjusted_speed_2 = std::clamp(adjusted_speed_2, 0, 255);

        set_wheel_motor_1_speed(static_cast<uint8_t>(adjusted_speed_1));
        set_wheel_motor_2_speed(static_cast<uint8_t>(adjusted_speed_2));

        prev_error1 = error1;
        prev_error2 = error2;

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
        } else {
            collision = true;
            break;
        }

        if (++steps > MAX_STEPS) {
            simulation_failed = true;
            break;
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

std::vector<RotationCandidate> analyze_pd_candidates(
    const std::vector<std::pair<std::vector<double>, RotationResult>>& trials)
{
    std::map<PdKey, std::vector<RotationResult>> grouped;

    for (const auto& t : trials) {
        const auto& v = t.first;

        if (v.empty()) continue;

        RotationConfig cfg = build_rotation_config(v);

        PdKey key {
            cfg.kp,
            cfg.kd,
            cfg.pid_shift
        };

        grouped[key].push_back(t.second);
    }

    std::vector<RotationCandidate> out;
    out.reserve(grouped.size());

    for (const auto& [key, results] : grouped) {
        RotationCandidate c;

        c.key = key;

        std::vector<double> times;
        std::vector<double> angles;
        std::vector<double> translations;

        times.reserve(results.size());
        angles.reserve(results.size());
        translations.reserve(results.size());

        int fail_count = 0;
        int coll_count = 0;

        for (const auto& r : results) {
            times.push_back(r.total_time);
            angles.push_back(r.final_angle_error);
            translations.push_back(r.total_translation);

            if (r.simulation_failed) fail_count++;
            if (r.collision) coll_count++;
        }

        c.time_stats = optimizer::compute_stats(times);
        c.angle_error_stats = optimizer::compute_stats(angles);
        c.translation_stats = optimizer::compute_stats(translations);

        const double n = static_cast<double>(results.size());
        c.failure_rate   = (n > 0) ? fail_count / n : 0.0;
        c.collision_rate = (n > 0) ? coll_count / n : 0.0;

        out.push_back(c);
    }

    return out;
}

void sort_rotation_candidates(std::vector<RotationCandidate>& v)
{
    constexpr double EPS {1e-6};

    std::sort(v.begin(), v.end(),
        [](const RotationCandidate& a, const RotationCandidate& b)
    {
        /* 1. HARD constraints first */
        if (std::abs(a.failure_rate - b.failure_rate) > EPS) {
            return a.failure_rate < b.failure_rate;
        }

        if (std::abs(a.collision_rate - b.collision_rate) > EPS) {
            return a.collision_rate < b.collision_rate;
        }

        /* 2. Accuracy (top priority) */
        if (std::abs(a.angle_error_stats.mean - b.angle_error_stats.mean) > EPS) {
            return a.angle_error_stats.mean < b.angle_error_stats.mean;
        }

        if (std::abs(a.translation_stats.mean - b.translation_stats.mean) > EPS) {
            return a.translation_stats.mean < b.translation_stats.mean;
        }

        /* 3. Speed */
        if (std::abs(a.time_stats.mean - b.time_stats.mean) > EPS) {
            return a.time_stats.mean < b.time_stats.mean;
        }

        /* 4. Stability (tie-breaker) */
        double a_var {a.time_stats.stddev
            + a.angle_error_stats.stddev
            + a.translation_stats.stddev};

        double b_var {b.time_stats.stddev
            + b.angle_error_stats.stddev
            + b.translation_stats.stddev};

        return a_var < b_var;
    });
}

std::vector<RotationCandidate>get_ranked_pd_candidates(
    const std::vector<std::pair<std::vector<double>, RotationResult>>& trials)
{
    auto candidates = analyze_pd_candidates(trials);
    sort_rotation_candidates(candidates);
    return candidates;
}

std::vector<optimizer::SweepParam> default_pd_sweep_params()
{
    return {
        {"motor_speed", 120, 220, 5},
        {"motor_speed_scale", 0.9, 1.1, 3},
        {"dt", 0.01, 0.5, 3},

        {"motor1_variance", -0.1, 0.1, 3},
        {"motor2_variance", -0.1, 0.1, 3},
        {"slip_factor", 0.9, 1.1, 3},
        {"wheel_circumference_scale", 0.95, 1.05, 3},
        {"wheel_base_scale", 0.95, 1.05, 3},

        {"kp", 0, 50, 51},
        {"kd", 0, 20, 21},
        {"pid_shift", 6, 10, 5}
    };
}

std::vector<std::pair<std::vector<double>, rotation::RotationResult>>
run_pd_sweep(const maze::Maze& maze, double target_angle)
{
    auto params {default_pd_sweep_params()};

    optimizer::SweepCursor cursor(params);
    std::vector<std::pair<std::vector<double>, rotation::RotationResult>> results;

    /* compute total number of combinations */
    size_t total {1};
    for (const auto& p : params) {
        total *= p.steps;
    }

    size_t count {0};
    constexpr size_t progress_increment {1};
    size_t current_progress {0};

    do {
        auto vals = cursor.values();

        auto cfg {rotation::build_rotation_config(vals)};
        auto result {rotation::run_rotation_simulation(maze, cfg, target_angle)};

        results.push_back({vals, result});

        count++;

        /* progress reporting */
        double percent {(100.0 * count) / total};

        if (percent >= current_progress) {
            std::cout << "Progress: " << current_progress << "%" << "\n";
            current_progress += progress_increment;
        }

    } while (cursor.next());

    return results;
}

void print_summary(const std::vector<std::pair<std::vector<double>, rotation::RotationResult>>& trials)
{
    auto summary {rotation::analyze_rotation_results(trials)};

    std::cout << std::setprecision(3);

    std::cout << "\n=== SUMMARY ===\n";
    std::cout << "Failure Rate   : " << summary.failure_rate << "\n";
    std::cout << "Collision Rate : " << summary.collision_rate << "\n";

    std::cout << "\nTranslation:\n";
    std::cout << "  mean=" << summary.translation_stats.mean
              << " std=" << summary.translation_stats.stddev
              << " min=" << summary.translation_stats.min
              << " max=" << summary.translation_stats.max << "\n";

    std::cout << "\nAngle Error:\n";
    std::cout << "  mean=" << summary.angle_error_stats.mean
              << " std=" << summary.angle_error_stats.stddev
              << " min=" << summary.angle_error_stats.min
              << " max=" << summary.angle_error_stats.max << "\n";
}

void print_top_candidates(const std::vector<std::pair<std::vector<double>,
        rotation::RotationResult>>& trials, int top_n)
{
    auto ranked {rotation::get_ranked_pd_candidates(trials)};

    std::cout << std::setprecision(3);

    std::cout << "\n=== TOP " << top_n << " CANDIDATES ===\n";

    std::cout
        << std::left
        << std::setw(6)  << "Rank"
        << std::setw(5)  << "kp"
        << std::setw(5)  << "kd"
        << std::setw(5)  << "sh"
        << std::setw(8)  << "Fail"
        << std::setw(8)  << "Coll"
        << std::setw(10) << "Angle"
        << std::setw(10) << "Trans"
        << std::setw(8)  << "Time"
        << "\n";

    for (int i {0}; i < std::min<int>(top_n, ranked.size()); i++) {
        const auto& c = ranked[i];

        std::cout
            << std::left
            << std::setw(6)  << (i + 1)
            << std::setw(5)  << c.key.kp
            << std::setw(5)  << c.key.kd
            << std::setw(5)  << c.key.shift
            << std::setw(8)  << c.failure_rate
            << std::setw(8)  << c.collision_rate
            << std::setw(10) << c.angle_error_stats.mean
            << std::setw(10) << c.translation_stats.mean
            << std::setw(8)  << c.time_stats.mean
            << "\n";
    }
}

void run_full_rotation_experiment(double target_angle, int top_n)
{
    std::vector<std::string> ascii {
        "+-+",
        "|S|",
        "+-+"
    };
    maze::Maze small_maze {maze::build_maze_from_ascii(ascii, 0.0)};
    
    std::cout << "Running rotation sweep...\n";

    auto trials {run_pd_sweep(small_maze, target_angle)};

    std::cout << "Trials: " << trials.size() << "\n";

    print_summary(trials);
    print_top_candidates(trials, top_n);
}

} /* rotation namespace */

/*----------------------------------------------------------------------------*/
/*                             Private Definitions                            */
/*----------------------------------------------------------------------------*/
/* none */
