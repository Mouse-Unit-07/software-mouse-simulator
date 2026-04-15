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
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>
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

MetricGlobalMax compute_global_max(const std::vector<Candidate>& candidates);
SingleCaseScoreBreakdown compute_single_case_score(const SingleCaseResultsMetrics& m,
                                                   const SingleCaseMetricGlobalMax& g);
ScoreBreakdown compute_score(const Candidate& c, const MetricGlobalMax& g);
double compute_total_primary(const ScoreBreakdown& s);
double compute_total_secondary(const ScoreBreakdown& s);

void write_summary(std::ofstream& out, size_t total_size);
void write_candidates_banner(std::ofstream& out);
void write_candidates(std::ofstream& out, const std::vector<Candidate>& candidates);

} /* unnamed namespace*/

/*----------------------------------------------------------------------------*/
/*                               Private Globals                              */
/*----------------------------------------------------------------------------*/
extern "C"
{

extern double ENCODER_TICKS_PER_MILLIMETER;

}

constexpr double FLOAT_TOLERANCE{1e-6};

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

ResultsMetrics compute_results_metrics(const std::vector<Result>& results)
{
    ResultsMetrics out;

    auto compute_single_case = [](const std::vector<Result>& results,
                                  auto accessor) -> SingleCaseResultsMetrics {
        SingleCaseResultsMetrics m;

        auto time = simulation_common::extract_metric(
            results, [&](const Result& r) { return accessor(r).total_time; });

        auto angle = simulation_common::extract_metric(
            results, [&](const Result& r) { return accessor(r).total_angle_error; });

        auto horiz = simulation_common::extract_metric(
            results, [&](const Result& r) { return accessor(r).total_horizontal_translation; });

        auto vert = simulation_common::extract_metric(
            results, [&](const Result& r) { return accessor(r).final_vertical_translation; });

        m.time_stats = simulation_common::compute_stats(time);
        m.angle_error_stats = simulation_common::compute_stats(angle);
        m.horizontal_translation_stats = simulation_common::compute_stats(horiz);
        m.vertical_translation_stats = simulation_common::compute_stats(vert);

        m.collision_rate = simulation_common::compute_rate(
            results, [&](const Result& r) { return accessor(r).collision; });

        m.timeout_rate = simulation_common::compute_rate(
            results, [&](const Result& r) { return accessor(r).timeout; });

        return m;
    };

    out.no_wall_metrics = compute_single_case(
        results, [](const Result& r) -> const SingleCaseResult& { return r.no_wall; });

    out.one_wall_metrics = compute_single_case(
        results, [](const Result& r) -> const SingleCaseResult& { return r.one_wall; });

    out.two_wall_metrics = compute_single_case(
        results, [](const Result& r) -> const SingleCaseResult& { return r.two_wall; });

    return out;
}

std::vector<Candidate> build_candidates(const std::vector<Trial>& trials)
{
    auto grouped = simulation_common::group_by(
        trials,
        [](const Trial& t) {
            return CandidateKey{
                t.config.single_wall_target,
                t.config.motor_speed,
                t.config.kp,
                t.config.kd,
                t.config.pid_shift,
                t.config.kp_ir,
                t.config.kd_ir};
        },
        [](const Trial& t) { return t.result; });

    std::vector<Candidate> out;
    out.reserve(grouped.size());

    for (const auto& [key, group_results] : grouped) {
        Candidate c;
        c.key = key;
        c.results_metrics = compute_results_metrics(group_results);
        out.push_back(c);
    }

    return out;
}

void score_and_sort_candidates(std::vector<Candidate>& candidates)
{
    auto g{compute_global_max(candidates)};

    for (auto& c : candidates) {
        c.score = compute_score(c, g);
    }

    std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) {
        double a_primary{compute_total_primary(a.score)};
        double b_primary{compute_total_primary(b.score)};

        if (std::abs(a_primary - b_primary) >= FLOAT_TOLERANCE) {
            return a_primary < b_primary;
        }

        double a_secondary{compute_total_secondary(a.score)};
        double b_secondary{compute_total_secondary(b.score)};

        return a_secondary < b_secondary;
    });
}

void write_analysis_to_file(const std::string& filename, const std::vector<Candidate>& candidates,
                            size_t total_size)
{
    std::ofstream out(filename);
    if (!out.is_open()) {
        throw std::runtime_error("Failed to open output file: " + filename);
    }

    out << std::fixed << std::setprecision(3);

    write_summary(out, total_size);

    out << "\n=== SORTED CANDIDATES ===\n";

    write_candidates_banner(out);
    write_candidates(out, candidates);
}

void run_full_move_forward_experiment(const std::string& filename, ConfigSweeper& sweeper)
{
    std::vector<Trial> trials;

    while (sweeper.next()) {
        Config cfg{sweeper.value()};

        Result result{move_forward::run_simulation(cfg)};

        trials.push_back({cfg, result});
    }

    auto candidates{build_candidates(trials)};
    score_and_sort_candidates(candidates);

    write_analysis_to_file(filename, candidates, trials.size());
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

MetricGlobalMax compute_global_max(const std::vector<Candidate>& candidates)
{
    auto compute_single_case_max = [&](auto accessor) {
        SingleCaseMetricGlobalMax g;

        for (const auto& c : candidates) {
            const auto& m{accessor(c)};

            g.time = std::max(g.time, simulation_common::collapse_metric(m.time_stats));
            g.angle = std::max(g.angle, simulation_common::collapse_metric(m.angle_error_stats));
            g.horizontal_translation =
                std::max(g.horizontal_translation,
                         simulation_common::collapse_metric(m.horizontal_translation_stats));
            g.vertical_translation =
                std::max(g.vertical_translation,
                         simulation_common::collapse_metric(m.vertical_translation_stats));
        }

        return g;
    };

    MetricGlobalMax g;

    g.no_wall_max = compute_single_case_max(
        [](const Candidate& c) { return c.results_metrics.no_wall_metrics; });

    g.one_wall_max = compute_single_case_max(
        [](const Candidate& c) { return c.results_metrics.one_wall_metrics; });

    g.two_wall_max = compute_single_case_max(
        [](const Candidate& c) { return c.results_metrics.two_wall_metrics; });

    return g;
}

SingleCaseScoreBreakdown compute_single_case_score(const SingleCaseResultsMetrics& m,
                                                   const SingleCaseMetricGlobalMax& g)
{
    SingleCaseScoreBreakdown s;

    double time{collapse_metric(m.time_stats)};
    double angle{collapse_metric(m.angle_error_stats)};
    double horiz{collapse_metric(m.horizontal_translation_stats)};
    double vert{collapse_metric(m.vertical_translation_stats)};

    s.time = simulation_common::compute_metric_score(time, g.time);
    s.angle = simulation_common::compute_metric_score(angle, g.angle);
    s.horizontal_translation =
        simulation_common::compute_metric_score(horiz, g.horizontal_translation);
    s.vertical_translation = simulation_common::compute_metric_score(vert, g.vertical_translation);

    s.collision = m.collision_rate;
    s.timeout = m.timeout_rate;

    /* PRIMARY: prioritize collision and timeout minimization */
    s.primary_total = s.collision + s.timeout;

    /* SECONDARY: performance metrics */
    s.secondary_total = s.time + s.angle + s.horizontal_translation + s.vertical_translation;

    return s;
}

ScoreBreakdown compute_score(const Candidate& c, const MetricGlobalMax& g)
{
    ScoreBreakdown s;

    s.no_wall_breakdown =
        compute_single_case_score(c.results_metrics.no_wall_metrics, g.no_wall_max);

    s.one_wall_breakdown =
        compute_single_case_score(c.results_metrics.one_wall_metrics, g.one_wall_max);

    s.two_wall_breakdown =
        compute_single_case_score(c.results_metrics.two_wall_metrics, g.two_wall_max);

    return s;
}

double compute_total_primary(const ScoreBreakdown& s)
{
    return s.no_wall_breakdown.primary_total 
           + s.one_wall_breakdown.primary_total
           + s.two_wall_breakdown.primary_total;
}

double compute_total_secondary(const ScoreBreakdown& s)
{
    return s.no_wall_breakdown.secondary_total
           + s.one_wall_breakdown.secondary_total
           + s.two_wall_breakdown.secondary_total;
}

void write_summary(std::ofstream& out, size_t total_size)
{
    out << "=== SUMMARY ===\n";
    out << "Total Size : " << total_size << "\n";
}

void write_candidates_banner(std::ofstream& out)
{
    out << std::left
        << std::setw(6)  << "#"
        << std::setw(6)  << "tg"
        << std::setw(6)  << "speed"
        << std::setw(6)  << "kp"
        << std::setw(6)  << "kd"
        << std::setw(8)  << "shift"
        << std::setw(10) << "kpir"
        << std::setw(10) << "kdir"

        /* GLOBAL TOTALS */
        << std::setw(12) << "P_tot"
        << std::setw(12) << "S_tot"

        /* PER CASE TOTALS */
        << std::setw(10) << "NW_P"
        << std::setw(10) << "NW_S"
        << std::setw(10) << "OW_P"
        << std::setw(10) << "OW_S"
        << std::setw(10) << "TW_P"
        << std::setw(10) << "TW_S"

        /* NO WALL */
        << std::setw(10) << "NW_t"
        << std::setw(10) << "NW_ang"
        << std::setw(10) << "NW_h"
        << std::setw(10) << "NW_v"
        << std::setw(8)  << "NW_col"
        << std::setw(8)  << "NW_to"

        /* ONE WALL */
        << std::setw(10) << "OW_t"
        << std::setw(10) << "OW_ang"
        << std::setw(10) << "OW_h"
        << std::setw(10) << "OW_v"
        << std::setw(8)  << "OW_col"
        << std::setw(8)  << "OW_to"

        /* TWO WALL */
        << std::setw(10) << "TW_t"
        << std::setw(10) << "TW_ang"
        << std::setw(10) << "TW_h"
        << std::setw(10) << "TW_v"
        << std::setw(8)  << "TW_col"
        << std::setw(8)  << "TW_to"

        << "\n";
}

void write_candidates(std::ofstream& out, const std::vector<Candidate>& candidates)
{
    auto fmt = [](double v) {
        std::ostringstream oss;
        oss << std::setprecision(3) << v;
        return oss.str();
    };

    auto pick = [](const SingleCaseResultsMetrics& m) {
        struct Flat {
            double t, ang, h, v;
            double col, to;
        };

        return Flat{simulation_common::collapse_metric(m.time_stats),
                    simulation_common::collapse_metric(m.angle_error_stats),
                    simulation_common::collapse_metric(m.horizontal_translation_stats),
                    simulation_common::collapse_metric(m.vertical_translation_stats),
                    m.collision_rate,
                    m.timeout_rate};
    };

    for (size_t i{0}; i < candidates.size(); ++i) {
        const auto& c{candidates[i]};

        const auto& s{c.score};

        /* totals */
        double p_total{compute_total_primary(s)};
        double s_total{compute_total_secondary(s)};

        /* per-case */
        const auto& nw_s{s.no_wall_breakdown};
        const auto& ow_s{s.one_wall_breakdown};
        const auto& tw_s{s.two_wall_breakdown};

        /* raw metrics */
        const auto nw{pick(c.results_metrics.no_wall_metrics)};
        const auto ow{pick(c.results_metrics.one_wall_metrics)};
        const auto tw{pick(c.results_metrics.two_wall_metrics)};

        out << std::left
            << std::setw(6)  << (i + 1)

            /* CONFIG */
            << std::setw(6)  << static_cast<int>(c.key.single_wall_target)
            << std::setw(6)  << static_cast<int>(c.key.motor_speed)
            << std::setw(6)  << c.key.kp
            << std::setw(6)  << c.key.kd
            << std::setw(8)  << c.key.pid_shift
            << std::setw(10) << c.key.kp_ir
            << std::setw(10) << c.key.kd_ir

            /* GLOBAL TOTALS */
            << std::setw(12) << fmt(p_total)
            << std::setw(12) << fmt(s_total)

            /* PER CASE TOTALS */
            << std::setw(10) << fmt(nw_s.primary_total)
            << std::setw(10) << fmt(nw_s.secondary_total)
            << std::setw(10) << fmt(ow_s.primary_total)
            << std::setw(10) << fmt(ow_s.secondary_total)
            << std::setw(10) << fmt(tw_s.primary_total)
            << std::setw(10) << fmt(tw_s.secondary_total)

            /* NO WALL */
            << std::setw(10) << fmt(nw.t)
            << std::setw(10) << fmt(nw.ang)
            << std::setw(10) << fmt(nw.h)
            << std::setw(10) << fmt(nw.v)
            << std::setw(8)  << fmt(nw.col)
            << std::setw(8)  << fmt(nw.to)

            /* ONE WALL */
            << std::setw(10) << fmt(ow.t)
            << std::setw(10) << fmt(ow.ang)
            << std::setw(10) << fmt(ow.h)
            << std::setw(10) << fmt(ow.v)
            << std::setw(8)  << fmt(ow.col)
            << std::setw(8)  << fmt(ow.to)

            /* TWO WALL */
            << std::setw(10) << fmt(tw.t)
            << std::setw(10) << fmt(tw.ang)
            << std::setw(10) << fmt(tw.h)
            << std::setw(10) << fmt(tw.v)
            << std::setw(8)  << fmt(tw.col)
            << std::setw(8)  << fmt(tw.to)

            << "\n";
    }
}

} /* unnamed namespace*/
