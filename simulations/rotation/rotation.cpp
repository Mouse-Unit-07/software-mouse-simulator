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
#include <fstream>
#include <sstream>
#include <stdexcept>
#include "point.hpp"
#include "ray.hpp"
#include "rectangular_hitbox.hpp"
#include "mouse.hpp"
#include "maze.hpp"
#include "simulation_common.hpp"
#include "rotation.hpp"

/*----------------------------------------------------------------------------*/
/*                            Private Declarations                            */
/*----------------------------------------------------------------------------*/
namespace
{

using namespace rotation;

bool dominates(const Candidate& a, const Candidate& b);
void write_summary(std::ofstream& out, const ResultsMetrics& overall_metrics, size_t total_size);
void write_candidates_banner(std::ofstream& out);
void write_candidates(std::ofstream& out, const std::vector<Candidate>& candidates);

void prepare_mock_for_rotation(const Config& cfg, const maze::Maze& maze, mouse::Mouse& mouse);
mouse_delta update_mock_by_dt(const Config& cfg, mouse::Mouse& mouse);
bool did_mouse_collide(const maze::Maze& maze, const mouse::Mouse& mouse);

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

ConfigSweeper::ConfigSweeper()
{
    indices = std::vector<size_t>(11, 0);
}

bool ConfigSweeper::next()
{
    if (first) {
        first = false;
        return true;
    }

    std::vector<size_t> sizes {
        motor_speed.size(),
        motor_speed_scale.size(),
        dt.size(),
        motor1_variance.size(),
        motor2_variance.size(),
        slip_factor.size(),
        wheel_circumference_scale.size(),
        wheel_base_scale.size(),
        kp.size(),
        kd.size(),
        pid_shift.size()
    };

    for (int i{static_cast<int>(indices.size()) - 1}; i >= 0; --i) {
        indices.at(i)++;

        if (indices.at(i) < sizes.at(i)) {
            return true;
        }

        indices.at(i) = 0;
    }

    return false;
}

Config ConfigSweeper::value() const
{
    Config cfg{};

    int i{0};

    cfg.motor_speed = motor_speed.at(indices.at(i++));
    cfg.motor_speed_scale = motor_speed_scale.at(indices.at(i++));
    cfg.dt = dt.at(indices.at(i++));

    cfg.motor1_variance = motor1_variance.at(indices.at(i++));
    cfg.motor2_variance = motor2_variance.at(indices.at(i++));
    cfg.slip_factor = slip_factor.at(indices.at(i++));
    cfg.wheel_circumference_scale = wheel_circumference_scale.at(indices.at(i++));
    cfg.wheel_base_scale = wheel_base_scale.at(indices.at(i++));

    cfg.kp = kp.at(indices.at(i++));
    cfg.kd = kd.at(indices.at(i++));
    cfg.pid_shift = pid_shift.at(indices.at(i++));

    return cfg;
}

Result run_simulation(const maze::Maze& maze, const Config& cfg, double target_angle)
{
    mouse::Mouse mouse;
    prepare_mock_for_rotation(cfg, maze, mouse);

    if (target_angle > 0) {
        set_wheel_motor_1_direction_backward();
        set_wheel_motor_2_direction_forward();
    } else {
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

    int32_t prev_error{0};
    double raw_target{std::abs(ENCODER_TICKS_PER_ROTATION_ANGLE_RADIANS * target_angle)};
    int32_t target_ticks{static_cast<int32_t>(raw_target)};

    while ((std::abs(get_encoder_1_ticks()) < target_ticks) || (std::abs(get_encoder_2_ticks()) < target_ticks)) {
        int32_t enc1{std::abs(get_encoder_1_ticks())};
        int32_t enc2{std::abs(get_encoder_2_ticks())};
    
        int32_t error{enc2 - enc1};
        int32_t derivative{error - prev_error};
        prev_error = error;
        int64_t p_term{static_cast<int64_t>(cfg.kp) * error};
        int64_t d_term{static_cast<int64_t>(cfg.kd) * derivative};
        int64_t control64{p_term + d_term};
        int32_t control{0};
        if (control64 >= 0) {
            control = static_cast<int32_t>(control64 >> cfg.pid_shift);
        } else {
            control = -static_cast<int32_t>((-control64) >> cfg.pid_shift);
        }

        int32_t base_speed{static_cast<int32_t>(cfg.motor_speed)};
        int32_t adjusted_speed_1{base_speed + control};
        int32_t adjusted_speed_2{base_speed - control};
        adjusted_speed_1 = std::clamp(adjusted_speed_1, 0, 255);
        adjusted_speed_2 = std::clamp(adjusted_speed_2, 0, 255);
        set_wheel_motor_1_speed(static_cast<uint8_t>(adjusted_speed_1));
        set_wheel_motor_2_speed(static_cast<uint8_t>(adjusted_speed_2));

        auto delta{update_mock_by_dt(cfg, mouse)};
        total_translation += sqrt((delta.dx * delta.dx) + (delta.dy * delta.dy));
        total_angle_rotation += delta.dtheta_rad;
        total_time += cfg.dt;

        if (did_mouse_collide(maze, mouse)) {
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
    int coll_count{0};
    int fail_count{0};

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

    const double n{static_cast<double>(results.size())};
    ResultsMetrics a;

    a.time_stats = simulation_common::compute_stats(time);
    a.angle_error_stats = simulation_common::compute_stats(angle);
    a.translation_stats = simulation_common::compute_stats(translation);
    a.timeout_rate = (n > 0) ? fail_count / n : 0.0;
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
            t.config.pid_shift,
            t.config.motor_speed
        };

        grouped[key].push_back(t.result);
    }

    std::vector<Candidate> out;
    out.reserve(grouped.size());

    for (const auto& [key, group_results] : grouped) {
        Candidate c;
        c.key = key;

        auto a{compute_results_metrics(group_results)};

        c.results_metrics = a;

        out.push_back(c);
    }

    return out;
}

std::vector<Candidate> compute_pareto_front(const std::vector<Candidate>& candidates)
{
    std::vector<Candidate> front;

    for (size_t i{0}; i < candidates.size(); ++i) {
        bool dominated{false};

        for (size_t j{0}; j < candidates.size(); ++j) {
            if (i == j) {
                continue;
            }

            if (dominates(candidates.at(j), candidates.at(i))) {
                dominated = true;
                break;
            }
        }

        if (!dominated) {
            front.push_back(candidates.at(i));
        }
    }

    return front;
}

void write_analysis_to_file(const std::string& filename, const std::vector<Candidate>& all_candidates,
        const std::vector<Candidate>& pareto_front, const ResultsMetrics& overall_metrics, size_t total_size)
{
    std::ofstream out(filename);
    if (!out.is_open()) {
        throw std::runtime_error("Failed to open output file: " + filename);
    }
    out << std::fixed << std::setprecision(3);

    write_summary(out, overall_metrics, total_size);
    
    out << "\n=== PARETO FRONT ===\n";
    write_candidates_banner(out);
    write_candidates(out, pareto_front);

    out << "\n=== ALL CANDIDATES ===\n";
    write_candidates_banner(out);
    write_candidates(out, all_candidates);
}

void run_full_rotation_experiment(const std::string& filename, double target_angle,
        ConfigSweeper& sweeper)
{
    std::vector<std::string> ascii{
        "+-+",
        "|S|",
        "+-+"
    };
    maze::Maze small_maze{maze::build_maze_from_ascii(ascii, 0.0)};
    std::vector<Trial> trials;
    std::vector<Result> all_results;

    while (sweeper.next()) {
        Config cfg {sweeper.value()};

        auto result{rotation::run_simulation(small_maze, cfg, target_angle)};

        trials.push_back({cfg, result});
        all_results.push_back(result);
    }

    auto overall_metrics{compute_results_metrics(all_results)};
    auto candidates{build_candidates(trials)};
    auto pareto_front{compute_pareto_front(candidates)};

    write_analysis_to_file(filename, candidates, pareto_front, overall_metrics, all_results.size());
}

} /* rotation namespace */

/*----------------------------------------------------------------------------*/
/*                             Private Definitions                            */
/*----------------------------------------------------------------------------*/
namespace
{

using namespace rotation;

bool dominates(const Candidate& a, const Candidate& b)
{
    const auto& A{a.results_metrics};
    const auto& B{b.results_metrics};

    bool strictly_better{false};

    constexpr double EPS{1e-4};
    constexpr double k{1.0};

    auto le = [&](double x, double y) {
        return x <= y;
    };

    auto lt = [&](double x, double y) {
        if (x < y) strictly_better = true;
        return x <= y;
    };

    auto score = [&](double mean, double stddev) {
        return mean + k * stddev;
    };

    /* hard constraints */
    if (!lt(A.timeout_rate,   B.timeout_rate))   return false;
    if (!lt(A.collision_rate, B.collision_rate)) return false;

    /* collapsed metrics */
    if (!lt(score(A.angle_error_stats.mean,   A.angle_error_stats.stddev),
            score(B.angle_error_stats.mean,   B.angle_error_stats.stddev))) return false;

    if (!lt(score(A.translation_stats.mean,   A.translation_stats.stddev),
            score(B.translation_stats.mean,   B.translation_stats.stddev))) return false;

    if (!lt(score(A.time_stats.mean,          A.time_stats.stddev),
            score(B.time_stats.mean,          B.time_stats.stddev))) return false;

    return strictly_better;
}

void write_summary(std::ofstream& out, const ResultsMetrics& overall_metrics, size_t total_size)
{
    out << "=== SUMMARY ===\n";
    out << "Total Size     : " << total_size << "\n";
    out << "Timeout Rate   : " << overall_metrics.timeout_rate << "\n";
    out << "Collision Rate : " << overall_metrics.collision_rate << "\n";

    out << "\nTime:\n";
    out << "  mean=" << overall_metrics.time_stats.mean
        << " std=" << overall_metrics.time_stats.stddev
        << " min=" << overall_metrics.time_stats.min
        << " max=" << overall_metrics.time_stats.max << "\n";

    out << "\nAngle Error:\n";
    out << "  mean=" << overall_metrics.angle_error_stats.mean
        << " std=" << overall_metrics.angle_error_stats.stddev
        << " min=" << overall_metrics.angle_error_stats.min
        << " max=" << overall_metrics.angle_error_stats.max << "\n";

    out << "\nTranslation:\n";
    out << "  mean=" << overall_metrics.translation_stats.mean
        << " std=" << overall_metrics.translation_stats.stddev
        << " min=" << overall_metrics.translation_stats.min
        << " max=" << overall_metrics.translation_stats.max << "\n";
}

void write_candidates_banner(std::ofstream& out)
{
    out << std::left
        << std::setw(6)  << "Rank"
        << std::setw(6)  << "kp"
        << std::setw(6)  << "kd"
        << std::setw(6)  << "sh"
        << std::setw(8)  << "speed"
        << std::setw(10) << "Timeout"
        << std::setw(10) << "Coll"
        << std::setw(38) << "Angle(m/sd/min/max)"
        << std::setw(28) << "Trans(m/sd/min/max)"
        << std::setw(28) << "Time(m/sd/min/max)"
        << "\n";
}

void write_candidates(std::ofstream& out, const std::vector<Candidate>& candidates)
{
    auto fmt_stats = [](const simulation_common::MetricStats& s) {
        std::ostringstream oss;
        oss << s.mean
            << "|" << s.stddev
            << "|" << s.min
            << "|" << s.max;
        return oss.str();
    };

    for (size_t i{0}; i < candidates.size(); ++i) {
        const auto& c {candidates.at(i)};

        out << std::left
            << std::setw(6)  << (i + 1)
            << std::setw(6)  << c.key.kp
            << std::setw(6)  << c.key.kd
            << std::setw(6)  << c.key.shift
            << std::setw(8)  << static_cast<int>(c.key.motor_speed)
            << std::setw(10) << c.results_metrics.timeout_rate
            << std::setw(10) << c.results_metrics.collision_rate
            << std::setw(38) << fmt_stats(c.results_metrics.angle_error_stats)
            << std::setw(28) << fmt_stats(c.results_metrics.translation_stats)
            << std::setw(28) << fmt_stats(c.results_metrics.time_stats)
            << "\n";
    }
}

void prepare_mock_for_rotation(const Config& cfg, const maze::Maze& maze, mouse::Mouse& mouse)
{
    reset_mock_device_drivers();
    set_motor_speed_scale(cfg.motor_speed_scale);
    set_motor_1_variance(cfg.motor1_variance);
    set_motor_2_variance(cfg.motor2_variance);
    set_motor_slip_factor(cfg.slip_factor);
    set_wheel_circumference_scale(cfg.wheel_circumference_scale);
    set_wheel_base_scale(cfg.wheel_base_scale);

    mouse.translate(maze.mouse_start.x, maze.mouse_start.y);
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

bool did_mouse_collide(const maze::Maze& maze, const mouse::Mouse& mouse)
{
    auto rc{maze::get_cell_from_point(maze, mouse.hitbox.center)};
    if (rc) {
        auto [r, c] {*rc};
        if (maze::does_hitbox_collide_in_vicinity(maze, mouse.hitbox, r, c)) {
            return true;
        }
    } else {
        return true;
    }

    return false;
}

} /* unnamed namespace */
