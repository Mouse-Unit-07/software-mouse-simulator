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
namespace
{

using namespace rotation;

struct RotationMetrics {
    std::vector<double> time;
    std::vector<double> angle;
    std::vector<double> translation;
};

RotationMetrics collect_metrics(const std::vector<RotationTrial>& trials);
RotationMetrics collect_metrics(const std::vector<RotationResult>& results);
RotationResultsMetrics compute_aggregate(const std::vector<RotationTrial>& trials);
RotationResultsMetrics compute_aggregate(const std::vector<RotationResult>& results);

} /* unnamed namespace */

/*----------------------------------------------------------------------------*/
/*                               Private Globals                              */
/*----------------------------------------------------------------------------*/
extern "C"
{

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
    double total_angle_rotation {0.0};
    double total_time {0.0};
    bool collision {false};
    bool simulation_failed {false};

    constexpr int MAX_STEPS {10000};
    int steps {0};

    int32_t prev_error {0};

    double raw_target {std::abs(ENCODER_TICKS_PER_ROTATION_ANGLE_RADIANS * target_angle)};
    int32_t target_ticks {static_cast<int32_t>(raw_target)};

    while ((std::abs(get_encoder_1_ticks()) < target_ticks) || (std::abs(get_encoder_2_ticks()) < target_ticks)) {
        int32_t enc1 = std::abs(get_encoder_1_ticks());
        int32_t enc2 = std::abs(get_encoder_2_ticks());
        
        int32_t error {enc2 - enc1};
        int32_t derivative {error - prev_error};

        int64_t p_term {static_cast<int64_t>(cfg.kp) * error};
        int64_t d_term {static_cast<int64_t>(cfg.kd) * derivative};
        int64_t control64 {p_term + d_term};

        int32_t control;
        if (control64 >= 0) {
            control = static_cast<int32_t>(control64 >> cfg.pid_shift);
        } else {
            control = -static_cast<int32_t>((-control64) >> cfg.pid_shift);
        }

        int32_t base_speed {static_cast<int32_t>(cfg.motor_speed)};
        int32_t adjusted_speed_1 {base_speed + control};
        int32_t adjusted_speed_2 {base_speed - control};
        adjusted_speed_1 = std::clamp(adjusted_speed_1, 0, 255);
        adjusted_speed_2 = std::clamp(adjusted_speed_2, 0, 255);

        set_wheel_motor_1_speed(static_cast<uint8_t>(adjusted_speed_1));
        set_wheel_motor_2_speed(static_cast<uint8_t>(adjusted_speed_2));

        prev_error = error;

        /* update virtual mouse */
        auto delta {compute_mouse_delta(mouse.hitbox.angle_rad, cfg.dt)};
        update_encoder_1_ticks(cfg.dt);
        update_encoder_2_ticks(cfg.dt);
        int32_t new_encoder_1_ticks {get_encoder_1_ticks()};
        int32_t new_encoder_2_ticks {get_encoder_2_ticks()};
        mouse.translate(delta.dx, delta.dy);
        mouse.rotate(delta.dtheta_rad);

        total_translation += sqrt(delta.dx * delta.dx + delta.dy * delta.dy);
        total_angle_rotation += delta.dtheta_rad;
        total_time += cfg.dt;

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

        steps++;
        if (steps > MAX_STEPS) {
            simulation_failed = true;
            break;
        }
    }

    return RotationResult{
        total_time,
        std::abs(target_angle - total_angle_rotation),
        total_translation,
        collision,
        simulation_failed
    };
}

RotationResultsMetrics analyze_rotation_results(const std::vector<RotationTrial>& trials)
{
    auto a = compute_aggregate(trials);

    return a;
}

std::vector<RotationCandidate> analyze_pd_candidates(const std::vector<RotationTrial>& trials)
{
    std::map<PdKey, std::vector<RotationResult>> grouped;

    for (const auto& t : trials) {
        if (t.configs.empty()) {
            continue;
        }

        auto cfg {build_rotation_config(t.configs)};

        PdKey key {
            cfg.kp,
            cfg.kd,
            cfg.pid_shift
        };

        grouped[key].push_back(t.result);
    }

    std::vector<RotationCandidate> out;
    out.reserve(grouped.size());

    for (const auto& [key, group_trials] : grouped) {
        RotationCandidate c;
        c.key = key;

        auto a {compute_aggregate(group_trials)};

        c.results_metrics = a;

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
        if (std::abs(a.results_metrics.failure_rate - b.results_metrics.failure_rate) > EPS) {
            return a.results_metrics.failure_rate < b.results_metrics.failure_rate;
        }

        if (std::abs(a.results_metrics.collision_rate - b.results_metrics.collision_rate) > EPS) {
            return a.results_metrics.collision_rate < b.results_metrics.collision_rate;
        }

        /* 2. Accuracy (top priority) */
        if (std::abs(a.results_metrics.angle_error_stats.mean - b.results_metrics.angle_error_stats.mean) > EPS) {
            return a.results_metrics.angle_error_stats.mean < b.results_metrics.angle_error_stats.mean;
        }

        if (std::abs(a.results_metrics.translation_stats.mean - b.results_metrics.translation_stats.mean) > EPS) {
            return a.results_metrics.translation_stats.mean < b.results_metrics.translation_stats.mean;
        }

        /* 3. Speed */
        if (std::abs(a.results_metrics.time_stats.mean - b.results_metrics.time_stats.mean) > EPS) {
            return a.results_metrics.time_stats.mean < b.results_metrics.time_stats.mean;
        }

        /* 4. Stability (tie-breaker) */
        double a_var {a.results_metrics.time_stats.stddev
            + a.results_metrics.angle_error_stats.stddev
            + a.results_metrics.translation_stats.stddev};

        double b_var {b.results_metrics.time_stats.stddev
            + b.results_metrics.angle_error_stats.stddev
            + b.results_metrics.translation_stats.stddev};

        return a_var < b_var;
    });
}

std::vector<RotationCandidate>get_ranked_pd_candidates(const std::vector<RotationTrial>& trials)
{
    auto candidates = analyze_pd_candidates(trials);
    sort_rotation_candidates(candidates);
    return candidates;
}

std::vector<optimizer::SweepConfig> default_pd_sweep_configs()
{
    return {
        {"motor_speed", 120, 220, 5}, // 120, 220, 5 | 100, 100, 1
        {"motor_speed_scale", 0.9, 1.1, 3}, // 0.9, 1.1, 3 | 1.0, 1.0, 1
        {"dt", 0.01, 0.5, 3}, // 0.01, 0.5, 3 | 0.001, 0.001, 1

        {"motor1_variance", -0.1, 0.1, 3}, // -0.1, 0.1, 3 | 0.0, 0.0, 1
        {"motor2_variance", -0.1, 0.1, 3}, // -0.1, 0.1, 3 | 0.0, 0.0, 1
        {"slip_factor", 0.9, 1.1, 3}, // 0.9, 1.1, 3 | 
        {"wheel_circumference_scale", 0.95, 1.05, 3}, // 0.95, 1.05, 3 | 1.0, 1.0, 1
        {"wheel_base_scale", 0.95, 1.05, 3}, // 0.95, 1.05, 3 | 1.0, 1.0, 1

        {"kp", 0, 4000, 21},
        {"kd", 0, 2000, 21},
        {"pid_shift", 8, 8, 1}
    };
}

std::vector<RotationTrial> run_pd_sweep(const maze::Maze& maze, double target_angle)
{
    auto configs {default_pd_sweep_configs()};

    optimizer::SweepCursor cursor(configs);
    std::vector<RotationTrial> results;

    do {
        auto vals = cursor.values();

        auto cfg {rotation::build_rotation_config(vals)};
        auto result {rotation::run_rotation_simulation(maze, cfg, target_angle)};

        results.push_back({vals, result});

    } while (cursor.next());

    return results;
}

void print_rotation_simulation_results(const std::vector<RotationTrial>& trials)
{
    auto summary {rotation::analyze_rotation_results(trials)};
    auto ranked {rotation::get_ranked_pd_candidates(trials)};

    std::cout << std::setprecision(3);

    std::cout << "\n=== SUMMARY ===\n";
    std::cout << "Failure Rate   : " << summary.failure_rate << "\n";
    std::cout << "Collision Rate : " << summary.collision_rate << "\n";

    std::cout << "\nTime:\n";
    std::cout << "  mean=" << summary.time_stats.mean
              << " std=" << summary.time_stats.stddev
              << " min=" << summary.time_stats.min
              << " max=" << summary.time_stats.max << "\n";

    std::cout << "\nAngle Error:\n";
    std::cout << "  mean=" << summary.angle_error_stats.mean
              << " std=" << summary.angle_error_stats.stddev
              << " min=" << summary.angle_error_stats.min
              << " max=" << summary.angle_error_stats.max << "\n";

    std::cout << "\nTranslation:\n";
    std::cout << "  mean=" << summary.translation_stats.mean
              << " std=" << summary.translation_stats.stddev
              << " min=" << summary.translation_stats.min
              << " max=" << summary.translation_stats.max << "\n";

    std::cout << "\n=== ALL CANDIDATES ===\n";

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

    for (int i {0}; i < ranked.size(); i++) {
        const auto& c = ranked[i];

        std::cout
            << std::left
            << std::setw(6)  << (i + 1)
            << std::setw(5)  << c.key.kp
            << std::setw(5)  << c.key.kd
            << std::setw(5)  << c.key.shift
            << std::setw(8)  << c.results_metrics.failure_rate
            << std::setw(8)  << c.results_metrics.collision_rate
            << std::setw(10) << c.results_metrics.angle_error_stats.mean
            << std::setw(10) << c.results_metrics.translation_stats.mean
            << std::setw(8)  << c.results_metrics.time_stats.mean
            << "\n";
    }
}

void run_full_rotation_experiment(double target_angle)
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

    print_rotation_simulation_results(trials);
}

} /* rotation namespace */

/*----------------------------------------------------------------------------*/
/*                             Private Definitions                            */
/*----------------------------------------------------------------------------*/
namespace
{

using namespace rotation;

RotationMetrics collect_metrics(const std::vector<RotationTrial>& trials)
{
    RotationMetrics m;

    m.time.reserve(trials.size());
    m.angle.reserve(trials.size());
    m.translation.reserve(trials.size());

    for (const auto& t : trials) {
        m.time.push_back(t.result.total_time);
        m.angle.push_back(t.result.final_angle_error);
        m.translation.push_back(t.result.total_translation);
    }

    return m;
}

RotationMetrics collect_metrics(const std::vector<RotationResult>& results)
{
    RotationMetrics m;

    m.time.reserve(results.size());
    m.angle.reserve(results.size());
    m.translation.reserve(results.size());

    for (const auto& r : results) {
        m.time.push_back(r.total_time);
        m.angle.push_back(r.final_angle_error);
        m.translation.push_back(r.total_translation);
    }

    return m;
}

RotationResultsMetrics compute_aggregate(const std::vector<RotationTrial>& trials)
{
    auto m {collect_metrics(trials)};

    RotationResultsMetrics a;

    a.time_stats = optimizer::compute_stats(m.time);
    a.angle_error_stats = optimizer::compute_stats(m.angle);
    a.translation_stats = optimizer::compute_stats(m.translation);

    a.failure_rate = optimizer::compute_rate(trials,
        [](const auto& t) { return t.result.simulation_failed; });

    a.collision_rate = optimizer::compute_rate(trials,
        [](const auto& t) { return t.result.collision; });

    return a;
}

RotationResultsMetrics compute_aggregate(const std::vector<RotationResult>& results)
{
    auto m {collect_metrics(results)};

    RotationResultsMetrics a;

    a.time_stats = optimizer::compute_stats(m.time);
    a.angle_error_stats = optimizer::compute_stats(m.angle);
    a.translation_stats = optimizer::compute_stats(m.translation);

    const double n {static_cast<double>(results.size())};

    int fail_count {0};
    int coll_count {0};

    for (const auto& r : results) {
        if (r.simulation_failed) fail_count++;
        if (r.collision) coll_count++;
    }

    a.failure_rate = (n > 0) ? fail_count / n : 0.0;
    a.collision_rate = (n > 0) ? coll_count / n : 0.0;

    return a;
}

} /* unnamed namespace */
