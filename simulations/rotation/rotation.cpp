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

extern double ENCODER_TICKS_PER_ROTATION_ANGLE_RADIANS;

}

/*----------------------------------------------------------------------------*/
/*                             Public Definitions                             */
/*----------------------------------------------------------------------------*/
namespace rotation
{

Config build_config(const std::vector<double>& v)
{
    Config cfg{};

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

Result run_simulation(const maze::Maze& maze, const Config& cfg, double target_angle)
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
    bool timeout {false};

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
            timeout = true;
            break;
        }
    }

    return Result{
        total_time,
        std::abs(target_angle - total_angle_rotation),
        total_translation,
        collision,
        timeout
    };
}

ResultsMetrics compute_results_metrics(const std::vector<Result>& results)
{
    std::vector<double> time;
    std::vector<double> angle;
    std::vector<double> translation;
    int coll_count {0};
    int fail_count {0};

    time.reserve(results.size());
    angle.reserve(results.size());
    translation.reserve(results.size());

    for (const auto& r : results) {
        time.push_back(r.total_time);
        angle.push_back(r.final_angle_error);
        translation.push_back(r.total_translation);
        if (r.collision) {
            coll_count++;
        }
        if (r.timeout) {
            fail_count++;
        }
    }

    const double n {static_cast<double>(results.size())};
    ResultsMetrics a;

    a.time_stats = optimizer::compute_stats(time);
    a.angle_error_stats = optimizer::compute_stats(angle);
    a.translation_stats = optimizer::compute_stats(translation);
    a.failure_rate = (n > 0) ? fail_count / n : 0.0;
    a.collision_rate = (n > 0) ? coll_count / n : 0.0;

    return a;
}

std::vector<Candidate> build_candidates(const std::vector<Trial>& trials)
{
    std::map<CandidateKey, std::vector<Result>> grouped;

    for (const auto& t : trials) {
        CandidateKey key {
            t.config.kp,
            t.config.kd,
            t.config.pid_shift
        };

        grouped[key].push_back(t.result);
    }

    std::vector<Candidate> out;
    out.reserve(grouped.size());

    for (const auto& [key, group_results] : grouped) {
        Candidate c;
        c.key = key;

        auto a {compute_results_metrics(group_results)};

        c.results_metrics = a;

        out.push_back(c);
    }

    return out;
}

void sort_candidates(std::vector<Candidate>& v)
{
    constexpr double EPS {1e-6};

    std::sort(v.begin(), v.end(),
        [](const Candidate& a, const Candidate& b)
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

void print_rotation_simulation_results(const std::vector<Candidate>& candidates, ResultsMetrics overall_metrics)
{
    std::cout << std::setprecision(3);

    std::cout << "\n=== SUMMARY ===\n";
    std::cout << "Failure Rate   : " << overall_metrics.failure_rate << "\n";
    std::cout << "Collision Rate : " << overall_metrics.collision_rate << "\n";

    std::cout << "\nTime:\n";
    std::cout << "  mean=" << overall_metrics.time_stats.mean
              << " std=" << overall_metrics.time_stats.stddev
              << " min=" << overall_metrics.time_stats.min
              << " max=" << overall_metrics.time_stats.max << "\n";

    std::cout << "\nAngle Error:\n";
    std::cout << "  mean=" << overall_metrics.angle_error_stats.mean
              << " std=" << overall_metrics.angle_error_stats.stddev
              << " min=" << overall_metrics.angle_error_stats.min
              << " max=" << overall_metrics.angle_error_stats.max << "\n";

    std::cout << "\nTranslation:\n";
    std::cout << "  mean=" << overall_metrics.translation_stats.mean
              << " std=" << overall_metrics.translation_stats.stddev
              << " min=" << overall_metrics.translation_stats.min
              << " max=" << overall_metrics.translation_stats.max << "\n";

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

    for (int i {0}; i < candidates.size(); i++) {
        const auto& c {candidates[i]};

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

void run_full_rotation_experiment(double target_angle, std::vector<optimizer::SweepConfig> configs)
{
    std::cout << "Running rotation sweep...\n";

    std::vector<std::string> ascii {
        "+-+",
        "|S|",
        "+-+"
    };
    maze::Maze small_maze {maze::build_maze_from_ascii(ascii, 0.0)};
    optimizer::SweepCursor cursor(configs);
    std::vector<Trial> trials;
    std::vector<Result> all_results;

    do {
        auto config_values = cursor.values();
        auto cfg {rotation::build_config(config_values)};
        auto result {rotation::run_simulation(small_maze, cfg, target_angle)};

        trials.push_back({cfg, result});
        all_results.push_back(result);

    } while (cursor.next());

    std::cout << "Trials: " << trials.size() << "\n";

    auto overall_metrics {compute_results_metrics(all_results)};
    auto candidates {build_candidates(trials)};
    sort_candidates(candidates);

    print_rotation_simulation_results(candidates, overall_metrics);
}

} /* rotation namespace */

/*----------------------------------------------------------------------------*/
/*                             Private Definitions                            */
/*----------------------------------------------------------------------------*/
/* none */
