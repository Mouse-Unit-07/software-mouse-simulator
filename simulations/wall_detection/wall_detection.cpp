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
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <SFML/Graphics.hpp>
#include <filesystem>
#include "point.hpp"
#include "ray.hpp"
#include "rectangular_hitbox.hpp"
#include "mouse.hpp"
#include "maze.hpp"
#include "visualizer.hpp"
#include "wall_detection.hpp"

/*----------------------------------------------------------------------------*/
/*                            Private Declarations                            */
/*----------------------------------------------------------------------------*/
namespace
{

std::optional<uint32_t> compute_ir_sensor_3_reading(const maze::Maze& maze,
        const mouse::Mouse& mouse);

} /* unnamed namespace */

/*----------------------------------------------------------------------------*/
/*                               Private Globals                              */
/*----------------------------------------------------------------------------*/
namespace
{

const std::string TEST_OUTPUT_DIRECTORY {"wall-detection-visualizer"};
bool visualizer_enabled {false};
visualizer::Visualizer local_visualizer;

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
    std::vector<std::string> ascii {
        "  +-+",
        "  |S|",
        "+-+ +",
        "|   |",
        "+-+-+"
    };
    maze::Maze test_maze {maze::build_maze_from_ascii(ascii, maze::OFFICIAL_POST_SIZE * (cfg.maze_size_scale - 1))};

    /* prepare mouse for wall detection */
    reset_mock_device_drivers();
    mouse::Mouse mouse;
    double MAX_HORIZONTAL_OFFSET {(maze::OFFICIAL_WALL_LENGTH_SIZE - mouse.hitbox.horizontal_size) / 2};
    double MAX_VERTICAL_OFFSET {(maze::OFFICIAL_WALL_LENGTH_SIZE - mouse.hitbox.vertical_size) / 2};
    mouse.rotate(cfg.mouse_angle);
    mouse.translate(
        test_maze.mouse_start.x + (MAX_HORIZONTAL_OFFSET * cfg.horizontal_position_variance),
        test_maze.mouse_start.y + (MAX_VERTICAL_OFFSET * cfg.vertical_position_variance)
    );

    if (visualizer_enabled) {
        local_visualizer.draw_maze(100.0f, test_maze);
        local_visualizer.draw_mouse_on_maze(mouse);
    }

    std::vector<bool> correct_detection_at_step;
    correct_detection_at_step.resize(cfg.total_steps);

    for (int i {0}; i < cfg.total_steps; i++) {
        auto potential_reading {compute_ir_sensor_3_reading(test_maze, mouse)};
        if (potential_reading.has_value()) {
            uint32_t reading {*potential_reading};
            long rounded_reading {std::lround(static_cast<double>(reading) * cfg.ir_reading_scale)};
            reading = static_cast<uint32_t>(std::clamp(rounded_reading, 0L, 1024L));
            correct_detection_at_step.at(i) = (reading < cfg.reading_threshold) ? true : false;
        } else {
            correct_detection_at_step.at(i) = false;
        }

        mouse.translate(0.0, test_maze.cell_size / cfg.total_steps);
    }

    if (visualizer_enabled) {
        local_visualizer.save_to_image_file(TEST_OUTPUT_DIRECTORY + "/" + config_to_string(cfg) + ".png");
    }

    return Result{
        std::move(correct_detection_at_step)
    };
}

ResultsMetrics compute_results_metrics(const std::vector<Result>& results)
{
    ResultsMetrics m;

    if (results.empty()) {
        return m;
    }

    const size_t steps {results.front().correct_detection_at_step.size()};

    // Build consensus signal
    std::vector<bool> consensus(steps, true);

    for (size_t t {0}; t < steps; ++t) {
        for (const auto& r : results) {
            if (!r.correct_detection_at_step[t]) {
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

} /* wall_detection namespace */

/*----------------------------------------------------------------------------*/
/*                             Private Definitions                            */
/*----------------------------------------------------------------------------*/
namespace
{

std::optional<uint32_t> compute_ir_sensor_3_reading(const maze::Maze& maze,
        const mouse::Mouse& mouse)
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
                local_visualizer.draw_ir_3_sensor_beam(mouse, distance);
            }
        }
    }

    return reading;
}

} /* unnamed namespace */
