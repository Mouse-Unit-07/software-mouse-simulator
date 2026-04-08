/*-------------------------------- FILE INFO ---------------------------------*/
/* Filename           : front_wall_detection.cpp                              */
/*                                                                            */
/* Implementation for micromouse front wall detection simulation and          */
/* associated config and results analysis helpers                             */
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
#include "infrared_sensor.h"

}

#include <cstdint>
#include <cmath>
#include <vector>
#include <string>
#include <map>
#include <iomanip>
#include <fstream>
#include <optional>
#include <algorithm>
#include "point.hpp"
#include "ray.hpp"
#include "rectangular_hitbox.hpp"
#include "mouse.hpp"
#include "maze.hpp"
#include "simulation_common.hpp"
#include "front_wall_detection.hpp"

/*----------------------------------------------------------------------------*/
/*                            Private Declarations                            */
/*----------------------------------------------------------------------------*/
namespace
{

using namespace front_wall_detection;

void prepare_mock_for_front_wall_detection(const Config& cfg, const maze::Maze& maze, mouse::Mouse& mouse);
std::optional<double> compute_ir_sensor_distance(const maze::Maze& maze,
        const mouse::Mouse& mouse, const geometry::Ray& ir_sensor);
uint32_t scale_and_clamp_ir_sensor_reading(uint32_t reading, const Config& cfg);

void write_summary(std::ofstream& out, const std::vector<Candidate>& candidates, size_t total_size);
void write_candidates(std::ofstream& out, const std::vector<Candidate>& candidates);

} /* unnamed namespace */

/*----------------------------------------------------------------------------*/
/*                               Private Globals                              */
/*----------------------------------------------------------------------------*/
/* none */

/*----------------------------------------------------------------------------*/
/*                             Public Definitions                             */
/*----------------------------------------------------------------------------*/
namespace front_wall_detection
{

bool ConfigSweeper::next()
{
    if (!initialized_) {
        sweeper.init_sizes({
            ir_reading_scale.size(),
            mouse_angle.size(),
            horizontal_position_variance.size(),
            vertical_position_variance.size(),
            reading_threshold.size()
        });

        initialized_ = true;
    }
    return sweeper.next();
}

Config ConfigSweeper::value() const
{
    const auto& idx {sweeper.get_indices()};
    int i{0};

    Config cfg{};

    cfg.ir_reading_scale = ir_reading_scale.at(idx.at(i++));
    cfg.mouse_angle = mouse_angle.at(idx.at(i++));
    cfg.horizontal_position_variance = horizontal_position_variance.at(idx.at(i++));
    cfg.vertical_position_variance = vertical_position_variance.at(idx.at(i++));
    cfg.reading_threshold = reading_threshold.at(idx.at(i++));

    return cfg;
}

Result run_simulation(const Config& cfg)
{
    std::vector<std::string> ascii_open{
        "+-+",
        "|S|",
        "+ +",
        "| |",
        "+-+"
    };
    maze::Maze open_maze{maze::build_maze_from_ascii(ascii_open, 0.0)};

    std::vector<std::string> ascii_closed{
        "+-+",
        "|S|",
        "+-+",
        "   ",
        "   "
    };
    maze::Maze closed_maze{maze::build_maze_from_ascii(ascii_closed, 0.0)};

    mouse::Mouse mouse;
    prepare_mock_for_front_wall_detection(cfg, open_maze, mouse);

    bool identified_absent_wall{false};
    bool identified_present_wall{false};
    uint32_t ir_1_reading{0u};
    uint32_t ir_4_reading{0u};
    uint32_t average_reading{0u};
    std::optional<double> potential_ir_1_distance{std::nullopt};
    std::optional<double> potential_ir_4_distance{std::nullopt};

    potential_ir_1_distance = compute_ir_sensor_distance(open_maze, mouse, mouse.ir_1_sensor);
    potential_ir_4_distance = compute_ir_sensor_distance(open_maze, mouse, mouse.ir_4_sensor);
    if (potential_ir_1_distance.has_value() && potential_ir_4_distance.has_value()) {
        update_ir_1_sensor_reading(*potential_ir_1_distance);
        update_ir_4_sensor_reading(*potential_ir_4_distance);
        ir_1_reading = read_ir_1_sensor();
        ir_1_reading = scale_and_clamp_ir_sensor_reading(ir_1_reading, cfg);
        ir_4_reading = read_ir_4_sensor();
        ir_4_reading = scale_and_clamp_ir_sensor_reading(ir_4_reading, cfg);
        average_reading = (ir_1_reading + ir_4_reading) / 2;
        identified_absent_wall = (average_reading < cfg.reading_threshold) ? true : false;
    } else {
        identified_absent_wall = false;
    }

    potential_ir_1_distance = compute_ir_sensor_distance(closed_maze, mouse, mouse.ir_1_sensor);
    potential_ir_4_distance = compute_ir_sensor_distance(closed_maze, mouse, mouse.ir_4_sensor);
    if (potential_ir_1_distance.has_value() && potential_ir_4_distance.has_value()) {
        update_ir_1_sensor_reading(*potential_ir_1_distance);
        update_ir_4_sensor_reading(*potential_ir_4_distance);
        ir_1_reading = read_ir_1_sensor();
        ir_1_reading = scale_and_clamp_ir_sensor_reading(ir_1_reading, cfg);
        ir_4_reading = read_ir_4_sensor();
        ir_4_reading = scale_and_clamp_ir_sensor_reading(ir_4_reading, cfg);
        average_reading = (ir_1_reading + ir_4_reading) / 2;
        identified_present_wall = (average_reading >= cfg.reading_threshold) ? true : false;
    } else {
        identified_present_wall = false;
    }

    return Result{
        identified_absent_wall,
        identified_present_wall
    };
}

ResultsMetrics compute_results_metrics(const std::vector<Result>& results)
{
    ResultsMetrics a;

    a.absent_wall_identification_rate = simulation_common::compute_rate(
        results, [](const Result& r){ return r.identified_absent_wall; }
    );
    a.present_wall_identification_rate = simulation_common::compute_rate(
        results, [](const Result& r){ return r.identified_present_wall; }
    );

    return a;
}

std::vector<Candidate> build_candidates(const std::vector<Trial>& trials)
{
    auto grouped = simulation_common::group_by(
        trials,
        [](const Trial& t) {
            return CandidateKey{t.config.reading_threshold};
        },
        [](const Trial& t) {
            return t.result;
        }
    );

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

std::vector<Candidate> sort_candidates_by_rate(const std::vector<Candidate>& candidates)
{
    constexpr double FLOAT_TOLERANCE{1e-6};
    std::vector<Candidate> out{candidates};

    auto score = [](const Candidate& c) {
        return (c.results_metrics.absent_wall_identification_rate +
                c.results_metrics.present_wall_identification_rate) / 2.0;
    };

    std::sort(out.begin(), out.end(),
        [&](const Candidate& a, const Candidate& b) {
            double sa{score(a)};
            double sb{score(b)};

            if (std::abs(sa - sb) > FLOAT_TOLERANCE) {
                return sa > sb;
            }

            return a.key.threshold < b.key.threshold;
        }
    );

    return out;
}

void write_analysis_to_file(const std::string& filename,
        const std::vector<Candidate>& candidates, size_t total_size)
{
    std::ofstream out(filename);
    if (!out.is_open()) {
        throw std::runtime_error("Failed to open output file: " + filename);
    }
    out << std::fixed << std::setprecision(3);

    write_summary(out, candidates, total_size);
    
    out << "\n=== ALL CANDIDATES ===\n";
    write_candidates(out, candidates);
}

void run_full_front_wall_detection_experiment(const std::string& filename, ConfigSweeper& sweeper)
{
    std::vector<Trial> trials;
    std::vector<Result> all_results;

    while (sweeper.next()) {
        Config cfg {sweeper.value()};

        auto result{run_simulation(cfg)};

        trials.push_back({cfg, result});
        all_results.push_back(result);
    }

    auto sorted_candidates{sort_candidates_by_rate(build_candidates(trials))};

    write_analysis_to_file(filename, sorted_candidates, all_results.size());
}

} /* front_wall_detection namespace */

/*----------------------------------------------------------------------------*/
/*                             Private Definitions                            */
/*----------------------------------------------------------------------------*/
namespace
{

using namespace front_wall_detection;

void prepare_mock_for_front_wall_detection(const Config& cfg, const maze::Maze& maze, mouse::Mouse& mouse)
{
    reset_mock_device_drivers();

    double max_horizontal_offset{(maze::OFFICIAL_WALL_LENGTH_SIZE - mouse.hitbox.horizontal_size) / 2};
    double max_vertical_offset{(maze::OFFICIAL_WALL_LENGTH_SIZE - mouse.hitbox.vertical_size) / 2};
    
    mouse.rotate(cfg.mouse_angle);
    mouse.translate(
        maze.mouse_start.x + (max_horizontal_offset * cfg.horizontal_position_variance),
        maze.mouse_start.y + (max_vertical_offset * cfg.vertical_position_variance)
    );
}

std::optional<double> compute_ir_sensor_distance(const maze::Maze& maze,
        const mouse::Mouse& mouse, const geometry::Ray& ir_sensor)
{
    std::optional<double> distance{std::nullopt};

    auto potential_rc{maze::get_cell_from_point(maze, mouse.hitbox.center)};
    if (potential_rc) {
        auto [r, c] {*potential_rc};
        auto potential_distance{maze::compute_ray_distance_in_vicinity(maze, ir_sensor, r, c)};
        if (potential_distance.has_value()) {
            distance = *potential_distance;
        }
    }

    return distance;
}

uint32_t scale_and_clamp_ir_sensor_reading(uint32_t reading, const Config& cfg)
{
    long rounded_reading{std::lround(static_cast<double>(reading) * cfg.ir_reading_scale)};
    return static_cast<uint32_t>(std::clamp(rounded_reading, 0L, 1024L));
}

void write_summary(std::ofstream& out, const std::vector<Candidate>& candidates, size_t total_size)
{
    out << "=== SUMMARY ===\n";
    out << "Total Trials : " << total_size << "\n";
    out << "Candidates   : " << candidates.size() << "\n\n";
}

void write_candidates(std::ofstream& out, const std::vector<Candidate>& candidates)
{
    out << std::left
        << std::setw(12) << "Threshold"
        << std::setw(10) << "Absent"
        << std::setw(10) << "Present"
        << "\n";

    for (const auto& c : candidates) {
        out << std::left
            << std::setw(12) << c.key.threshold
            << std::setw(10) << c.results_metrics.absent_wall_identification_rate
            << std::setw(10) << c.results_metrics.present_wall_identification_rate
            << "\n";
    }
}

} /* unnamed namespace */
