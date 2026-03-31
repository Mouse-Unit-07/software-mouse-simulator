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
#include <map>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <memory>
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

void prepare_mock_for_wall_detection(const Config& cfg, const maze::Maze& maze, mouse::Mouse& mouse);
std::optional<uint32_t> compute_ir_sensor_3_reading(const maze::Maze& maze,
        const mouse::Mouse& mouse, visualizer::Visualizer& visualizer);

DetectionWindow find_window_with_rate(const ResultsMetrics& m, double required_rate);

void write_summary(std::ofstream& out, const std::vector<Candidate>& candidates, size_t total_size);
void write_candidates(std::ofstream& out, const std::vector<Candidate>& candidates);

} /* unnamed namespace */

/*----------------------------------------------------------------------------*/
/*                               Private Globals                              */
/*----------------------------------------------------------------------------*/
namespace
{

const std::string TEST_OUTPUT_DIRECTORY{"wall-detection-visualizer"};
bool visualizer_enabled{false};
visualizer::Visualizer wall_absent_visualizer;
visualizer::Visualizer wall_present_visualizer;

} /* unnamed namespace */

/*----------------------------------------------------------------------------*/
/*                             Public Definitions                             */
/*----------------------------------------------------------------------------*/
namespace wall_detection
{

ConfigSweeper::ConfigSweeper()
{
    indices = std::vector<size_t>(7, 0);
}

bool ConfigSweeper::next()
{
    if (first) {
        first = false;
        return true;
    }

    std::vector<size_t> sizes {
        maze_size_scale.size(),
        ir_reading_scale.size(),
        mouse_angle.size(),
        horizontal_position_variance.size(),
        vertical_position_variance.size(),
        total_steps.size(),
        reading_threshold.size()
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

    cfg.maze_size_scale = maze_size_scale.at(indices.at(i++));
    cfg.ir_reading_scale = ir_reading_scale.at(indices.at(i++));
    cfg.mouse_angle = mouse_angle.at(indices.at(i++));
    cfg.horizontal_position_variance = horizontal_position_variance.at(indices.at(i++));
    cfg.vertical_position_variance = vertical_position_variance.at(indices.at(i++));
    cfg.total_steps = total_steps.at(indices.at(i++));

    cfg.reading_threshold = reading_threshold.at(indices.at(i++));

    return cfg;
}


void enable_visualization(void)
{
    visualizer_enabled = true;
}

void disable_visualization(void)
{
    visualizer_enabled = false;
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
    std::vector<std::string> ascii_open{
        "  +-+",
        "  |S|",
        "+-+ +",
        "|   |",
        "+-+-+"
    };maze::Maze open_maze{maze::build_maze_from_ascii(ascii_open, maze::OFFICIAL_POST_SIZE * (cfg.maze_size_scale - 1))};

    std::vector<std::string> ascii_closed{
        "  +-+",
        "  |S|",
        "  + +",
        "  | |",
        "  +-+"
    };
    maze::Maze closed_maze{maze::build_maze_from_ascii(ascii_closed, maze::OFFICIAL_POST_SIZE * (cfg.maze_size_scale - 1))};

    /* prepare mouse for wall detection */
    mouse::Mouse mouse;
    prepare_mock_for_wall_detection(cfg, open_maze, mouse);

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

    for (int i{0}; i < cfg.total_steps; i++) {
        auto potential_reading_1{compute_ir_sensor_3_reading(open_maze, mouse, wall_absent_visualizer)};
        if (potential_reading_1.has_value()) {
            uint32_t reading{*potential_reading_1};
            long rounded_reading{std::lround(static_cast<double>(reading) * cfg.ir_reading_scale)};
            reading = static_cast<uint32_t>(std::clamp(rounded_reading, 0L, 1024L));
            wall_absent_at_step.at(i) = (reading < cfg.reading_threshold) ? true : false;
        } else {
            wall_absent_at_step.at(i) = false;
        }

        auto potential_reading_2{compute_ir_sensor_3_reading(closed_maze, mouse, wall_present_visualizer)};
        if (potential_reading_2.has_value()) {
            uint32_t reading{*potential_reading_2};
            long rounded_reading{std::lround(static_cast<double>(reading) * cfg.ir_reading_scale)};
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

    const size_t steps{results.front().wall_absent_at_step.size()};

    std::vector<int> agreements(steps, 0);

    for (size_t t{0}; t < steps; ++t) {
        for (const auto& r : results) {
            if (r.wall_absent_at_step.at(t) && r.wall_present_at_step.at(t)) {
                agreements.at(t)++;
            }
        }
    }

    m.correct_detection_count_at_step = std::move(agreements);
    m.total_detection_counts_per_step = results.size();

    return m;
}

std::vector<Candidate> build_candidates(const std::vector<Trial>& trials)
{
    std::map<CandidateKey, std::vector<Result>> grouped;

    for (const auto& t : trials) {
        CandidateKey key{t.config.reading_threshold};
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

std::vector<Candidate> filter_candidates_by_rate(const std::vector<Candidate>& candidates,
        double required_rate)
{
    std::vector<Candidate> out;

    for (const auto& c : candidates) {
        auto [start, size] = find_window_with_rate(c.results_metrics, required_rate);
        if (size > 0) {
            Candidate copy = c;
            copy.results_metrics.detection_window.window_start = start;
            copy.results_metrics.detection_window.window_size = size;
            out.push_back(copy);
        }
    }

    return out;
}

void write_analysis_to_file(const std::string& filename, const std::vector<Candidate>& candidates,
        size_t total_size, double min_correct_rate)
{
    std::ofstream out(filename);
    if (!out.is_open()) {
        throw std::runtime_error("Failed to open file: " + filename);
    }

    write_summary(out, candidates, total_size);

    /* sweep agreement rates: 100%, 95%, ..., down to cutoff */
    for (double rate{1.0}; rate >= min_correct_rate; rate -= 0.05) {
        out << "\n=== CORRECT DETECTION RATE >= " << (rate * 100.0) << "% ===\n";

        auto filtered{filter_candidates_by_rate(candidates, rate)};

        write_candidates(out, filtered);
    }
}

void run_full_wall_detection_experiment(const std::string& filename,
        ConfigSweeper& sweeper, double min_correct_rate)
{
    std::vector<Trial> trials;
    std::vector<Result> all_results;

    while (sweeper.next()) {
        Config cfg{sweeper.value()};

        auto result{run_simulation(cfg)};

        trials.push_back({cfg, result});
        all_results.push_back(result);
    }

    auto candidates{build_candidates(trials)};

    write_analysis_to_file(filename, candidates, all_results.size(), min_correct_rate);
}

} /* wall_detection namespace */

/*----------------------------------------------------------------------------*/
/*                             Private Definitions                            */
/*----------------------------------------------------------------------------*/
namespace
{

using namespace wall_detection;

void prepare_mock_for_wall_detection(const Config& cfg, const maze::Maze& maze, mouse::Mouse& mouse)
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

std::optional<uint32_t> compute_ir_sensor_3_reading(const maze::Maze& maze,
        const mouse::Mouse& mouse, visualizer::Visualizer& visualizer)
{
    std::optional<uint32_t> reading{std::nullopt};

    auto potential_rc{maze::get_cell_from_point(maze, mouse.hitbox.center)};
    if (potential_rc) {
        auto [r, c] {*potential_rc};
        auto potential_distance{maze::compute_ray_distance_in_vicinity(maze, mouse.ir_3_sensor, r, c)};
        if (potential_distance.has_value()) {
            auto distance{*potential_distance};
            update_ir_3_sensor_reading(distance);
            reading = read_ir_3_sensor();

            if (visualizer_enabled) {
                visualizer.draw_ir_3_sensor_beam(mouse, distance);
            }
        }
    }

    return reading;
}

DetectionWindow find_window_with_rate(const ResultsMetrics& m, double required_rate)
{
    int best_start{-1};
    int best_size{0};

    int current_start{-1};
    int current_size{0};

    const size_t steps{m.correct_detection_count_at_step.size()};

    for (size_t t{0}; t < steps; ++t) {
        double rate{static_cast<double>(m.correct_detection_count_at_step.at(t)) / m.total_detection_counts_per_step};

        if (rate >= required_rate) {
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

    return {best_start, best_size};
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
        << std::setw(14) << "WindowStart"
        << std::setw(12) << "WindowSize"
        << "\n";

    for (const auto& c : candidates) {
        out << std::left
            << std::setw(12) << c.key.threshold
            << std::setw(14) << c.results_metrics.detection_window.window_start
            << std::setw(12) << c.results_metrics.detection_window.window_size
            << "\n";
    }
}

} /* unnamed namespace */
