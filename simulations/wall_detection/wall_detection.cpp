/*-------------------------------- FILE INFO ---------------------------------*/
/* Filename           : wall_detection.cpp                                    */
/*                                                                            */
/* Implementation for micromouse wall detection simulation and associated     */
/* config and results analysis helpers                                        */
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

#include <vector>
#include <string>
#include <optional>
#include <functional>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <SFML/Graphics.hpp>
#include <filesystem>
#include "point.hpp"
#include "ray.hpp"
#include "rectangular_hitbox.hpp"
#include "mouse.hpp"
#include "maze.hpp"
#include "visualizer.hpp"
#include "simulation_common.hpp"
#include "wall_detection.hpp"

/*----------------------------------------------------------------------------*/
/*                            Private Declarations                            */
/*----------------------------------------------------------------------------*/
namespace
{

using namespace wall_detection;

std::optional<uint32_t> compute_ir_sensor_3_reading(const maze::Maze& maze,
        const mouse::Mouse& mouse, visualizer::Visualizer& visualizer);
void write_summary(std::ofstream& out, const std::vector<Candidate>& candidates, size_t total_size);
void write_candidates(std::ofstream& out, const std::vector<Candidate>& candidates);

} /* unnamed namespace */

/*----------------------------------------------------------------------------*/
/*                               Private Globals                              */
/*----------------------------------------------------------------------------*/
namespace
{

const std::string TEST_OUTPUT_DIRECTORY {"wall-detection-visualizer"};
bool visualizer_enabled {false};
visualizer::Visualizer wall_absent_visualizer;
visualizer::Visualizer wall_present_visualizer;

} /* unnamed namespace */

/*----------------------------------------------------------------------------*/
/*                             Public Definitions                             */
/*----------------------------------------------------------------------------*/
namespace wall_detection
{

void enable_visualization(void)
{
    visualizer_enabled = true;
}

void disable_visualization(void)
{
    visualizer_enabled = false;
}

Config build_config(const std::vector<double>& v)
{
    Config cfg{};

    int i {0};

    cfg.maze_size_scale = v[i++];
    cfg.ir_reading_scale = v[i++];
    cfg.mouse_angle = v[i++];
    cfg.horizontal_position_variance = v[i++];
    cfg.vertical_position_variance = v[i++];
    cfg.total_steps = v[i++];
    cfg.reading_threshold = v[i++];

    return cfg;
}

std::string config_to_string(const Config& cfg)
{
    auto fmt = [](double v, int precision = 2) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(precision) << v;
        return oss.str();
    };

    auto sanitize = [](std::string s) {
        for (char& c : s) {
            if (c == '.') c = 'p';
            else if (c == '-') c = 'n';
        }
        return s;
    };

    auto encode = [&](double v) {
        return sanitize(fmt(v));
    };

    std::ostringstream oss;

    oss << encode(cfg.maze_size_scale) << "-"
        << encode(cfg.ir_reading_scale) << "-"
        << encode(cfg.mouse_angle) << "-"
        << encode(cfg.horizontal_position_variance) << "-"
        << encode(cfg.vertical_position_variance) << "-"
        << cfg.total_steps << "-"
        << encode(cfg.reading_threshold);

    return oss.str();
}

Result run_simulation(const Config& cfg)
{
    if (visualizer_enabled) {
        std::filesystem::create_directories(TEST_OUTPUT_DIRECTORY);
    }

    /* create maze */
    std::vector<std::string> ascii_open {
        "  +-+",
        "  |S|",
        "+-+ +",
        "|   |",
        "+-+-+"
    };maze::Maze open_maze {maze::build_maze_from_ascii(ascii_open, maze::OFFICIAL_POST_SIZE * (cfg.maze_size_scale - 1))};

    std::vector<std::string> ascii_closed {
        "  +-+",
        "  |S|",
        "  + +",
        "  | |",
        "  +-+"
    };
    maze::Maze closed_maze {maze::build_maze_from_ascii(ascii_closed, maze::OFFICIAL_POST_SIZE * (cfg.maze_size_scale - 1))};

    /* prepare mouse for wall detection */
    reset_mock_device_drivers();
    mouse::Mouse mouse;
    double MAX_HORIZONTAL_OFFSET {(maze::OFFICIAL_WALL_LENGTH_SIZE - mouse.hitbox.horizontal_size) / 2};
    double MAX_VERTICAL_OFFSET {(maze::OFFICIAL_WALL_LENGTH_SIZE - mouse.hitbox.vertical_size) / 2};
    mouse.rotate(cfg.mouse_angle);
    mouse.translate(
        open_maze.mouse_start.x + (MAX_HORIZONTAL_OFFSET * cfg.horizontal_position_variance),
        open_maze.mouse_start.y + (MAX_VERTICAL_OFFSET * cfg.vertical_position_variance)
    );

    if (visualizer_enabled) {
        wall_absent_visualizer.draw_maze(100.0f, open_maze);
        wall_present_visualizer.draw_maze(100.0f, closed_maze);
        wall_absent_visualizer.draw_mouse_on_maze(mouse);
        wall_present_visualizer.draw_mouse_on_maze(mouse);
    }

    std::vector<bool> wall_absent_at_step;
    std::vector<bool> wall_present_at_step;
    wall_absent_at_step.resize(cfg.total_steps);
    wall_present_at_step.resize(cfg.total_steps);

    for (int i {0}; i < cfg.total_steps; i++) {
        auto potential_reading_1 {compute_ir_sensor_3_reading(open_maze, mouse, wall_absent_visualizer)};
        if (potential_reading_1.has_value()) {
            uint32_t reading {*potential_reading_1};
            long rounded_reading {std::lround(static_cast<double>(reading) * cfg.ir_reading_scale)};
            reading = static_cast<uint32_t>(std::clamp(rounded_reading, 0L, 1024L));
            wall_absent_at_step.at(i) = (reading < cfg.reading_threshold) ? true : false;
        } else {
            wall_absent_at_step.at(i) = false;
        }

        auto potential_reading_2 {compute_ir_sensor_3_reading(closed_maze, mouse, wall_present_visualizer)};
        if (potential_reading_2.has_value()) {
            uint32_t reading {*potential_reading_2};
            long rounded_reading {std::lround(static_cast<double>(reading) * cfg.ir_reading_scale)};
            reading = static_cast<uint32_t>(std::clamp(rounded_reading, 0L, 1024L));
            wall_present_at_step.at(i) = (reading >= cfg.reading_threshold) ? true : false;
        } else {
            wall_present_at_step.at(i) = false;
        }

        mouse.translate(0.0, open_maze.cell_size / cfg.total_steps);
    }

    if (visualizer_enabled) {
        wall_absent_visualizer.save_to_image_file(TEST_OUTPUT_DIRECTORY + "/" + config_to_string(cfg) + "-wa.png");
        wall_present_visualizer.save_to_image_file(TEST_OUTPUT_DIRECTORY + "/" + config_to_string(cfg) + "-wp.png");
    }

    return Result{
        std::move(wall_absent_at_step),
        std::move(wall_present_at_step),
    };
}

ResultsMetrics compute_results_metrics(const std::vector<Result>& results)
{
    ResultsMetrics m;

    if (results.empty()) {
        return m;
    }

    const size_t steps {results.front().wall_absent_at_step.size()};

    // Build consensus signal
    std::vector<bool> consensus(steps, true);

    for (size_t t {0}; t < steps; ++t) {
        for (const auto& r : results) {
            if (!r.wall_absent_at_step[t] || !r.wall_present_at_step[t]) {
                consensus[t] = false;
                break;
            }
        }
    }

    // Find longest contiguous true segment
    int best_start {-1};
    int best_size {0};

    int current_start {-1};
    int current_size {0};

    for (size_t t {0}; t < steps; ++t) {
        if (consensus[t]) {
            if (current_size == 0) {
                current_start = static_cast<int>(t);
            }
            current_size++;

            if (current_size > best_size) {
                best_size = current_size;
                best_start = current_start;
            }
        } else {
            current_size = 0;
        }
    }

    m.window_start = best_start;
    m.window_size = best_size;

    return m;
}

std::vector<Candidate> build_candidates(const std::vector<Trial>& trials)
{
    std::map<CandidateKey, std::vector<Result>> grouped;

    for (const auto& t : trials) {
        CandidateKey key {t.config.reading_threshold};
        grouped[key].push_back(t.result);
    }

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

void sort_candidates_by_lowest_threshold(std::vector<Candidate>& candidates)
{
    std::sort(candidates.begin(), candidates.end(),
        [](const Candidate& a, const Candidate& b) {
            return a.key.threshold < b.key.threshold;
        });
}

void write_analysis_to_file(const std::string& filename,
        const std::vector<Candidate>& sorted_candidates, size_t total_size)
{
    std::ofstream out(filename);
    if (!out.is_open()) {
        throw std::runtime_error("Failed to open file: " + filename);
    }

    write_summary(out, sorted_candidates, total_size);
    write_candidates(out, sorted_candidates);
}

void run_full_wall_detection_experiment(const std::string& filename,
        std::vector<simulation_common::SweepConfig> configs)
{
    simulation_common::SweepCursor cursor(configs);

    std::vector<Trial> trials;
    std::vector<Result> all_results;

    do {
        auto values {cursor.values()};

        auto cfg {build_config(values)};
        auto result {run_simulation(cfg)};

        trials.push_back({cfg, result});
        all_results.push_back(result);

    } while (cursor.next());

    auto candidates {build_candidates(trials)};
    sort_candidates_by_lowest_threshold(candidates);

    write_analysis_to_file(filename, candidates, all_results.size());
}

} /* wall_detection namespace */

/*----------------------------------------------------------------------------*/
/*                             Private Definitions                            */
/*----------------------------------------------------------------------------*/
namespace
{

using namespace wall_detection;

std::optional<uint32_t> compute_ir_sensor_3_reading(const maze::Maze& maze,
        const mouse::Mouse& mouse, visualizer::Visualizer& visualizer)
{
    std::optional<uint32_t> reading{std::nullopt};

    auto potential_rc {maze::get_cell_from_point(maze, mouse.hitbox.center)};
    if (potential_rc) {
        auto [r, c] {*potential_rc};
        auto potential_distance {maze::compute_ray_distance_in_vicinity(maze, mouse.ir_3_sensor, r, c)};
        if (potential_distance.has_value()) {
            auto distance {*potential_distance};
            update_ir_3_sensor_reading(distance);
            reading = read_ir_3_sensor();

            if (visualizer_enabled) {
                visualizer.draw_ir_3_sensor_beam(mouse, distance);
            }
        }
    }

    return reading;
}

void write_summary(std::ofstream& out, const std::vector<Candidate>& candidates, size_t total_size)
{
    out << "=== SUMMARY ===\n";
    out << "Total Trials : " << total_size << "\n";
    out << "Candidates   : " << candidates.size() << "\n\n";

    // find best candidate (first valid window)
    for (const auto& c : candidates) {
        if (c.results_metrics.window_size > 0) {
            out << "Best Threshold : " << c.key.threshold << "\n";
            out << "Window Start   : " << c.results_metrics.window_start << "\n";
            out << "Window Size    : " << c.results_metrics.window_size << "\n\n";
            return;
        }
    }

    out << "No valid detection window found.\n\n";
}

void write_candidates(std::ofstream& out, const std::vector<Candidate>& candidates)
{
    out << "=== CANDIDATES ===\n";

    out << std::left
        << std::setw(12) << "Threshold"
        << std::setw(14) << "WindowStart"
        << std::setw(12) << "WindowSize"
        << "\n";

    for (const auto& c : candidates) {
        out << std::left
            << std::setw(12) << c.key.threshold
            << std::setw(14) << c.results_metrics.window_start
            << std::setw(12) << c.results_metrics.window_size
            << "\n";
    }
}

} /* unnamed namespace */
