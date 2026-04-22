/*-------------------------------- FILE INFO ---------------------------------*/
/* Filename           : maze.cpp                                              */
/*                                                                            */
/* Implementation for maze building logic for micromouse simulations          */
/*                                                                            */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*                               Include Files                                */
/*----------------------------------------------------------------------------*/
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include "point.hpp"
#include "ray.hpp"
#include "rectangular_hitbox.hpp"
#include "interactions.hpp"
#include "maze.hpp"

/*----------------------------------------------------------------------------*/
/*                            Private Declarations                            */
/*----------------------------------------------------------------------------*/
namespace
{

using namespace maze;

std::string validate_ascii_maze(const std::vector<std::string>& ascii);

geometry::RectangularHitbox create_post(const geometry::Point& center, double size_adjustment);
geometry::RectangularHitbox create_vertical_wall(const geometry::Point& center,
                                                 double size_adjustment);
geometry::RectangularHitbox create_horizontal_wall(const geometry::Point& center,
                                                   double size_adjustment);

geometry::Point ascii_to_world(int r, int c, double size_adjustment);
void attach_to_cell(Maze& maze, size_t obstacle_index, int row, int col);
void attach_vertical_wall_cells(Maze& maze, size_t obstacle_index, int r, int c);
void attach_horizontal_wall_cells(Maze& maze, size_t obstacle_index, int r, int c);
void attach_post_cells(Maze& maze, size_t obstacle_index, int r, int c);

std::optional<std::pair<int, int>> get_cell_from_point(const Maze& maze, const geometry::Point& p);
bool does_hitbox_collide_in_vicinity(const Maze& maze, const geometry::RectangularHitbox& hitbox,
                                     int row, int col);
std::optional<double> compute_ray_distance_in_cell(const Maze& maze, const geometry::Ray& ray,
                                                   int row, int col);
std::optional<double> compute_ray_distance_in_vicinity(const Maze& maze, const geometry::Ray& ray,
                                                       int row, int col);
std::optional<double> compute_ir_sensor_distance(const maze::Maze& maze,
                                                 const geometry::Point& point,
                                                 const geometry::Ray& ir_sensor);

} /* unnamed namespace */

/*----------------------------------------------------------------------------*/
/*                               Private Globals                              */
/*----------------------------------------------------------------------------*/
/* none */

/*----------------------------------------------------------------------------*/
/*                             Public Definitions                             */
/*----------------------------------------------------------------------------*/
namespace maze
{

const Cell& Maze::get_cell(int row, int col) const
{
    return cells.at(row * cols + col);
}

Maze build_maze_from_ascii(const std::vector<std::string>& ascii, double obstacle_size_adjustment)
{
    const std::string error{validate_ascii_maze(ascii)};
    if (!error.empty()) {
        throw std::invalid_argument(error);
    }

    Maze maze{};

    int ascii_rows{static_cast<int>(ascii.size())};
    int ascii_cols{static_cast<int>(ascii.at(0).size())};

    maze.rows = (ascii_rows - 1) / 2;
    maze.cols = (ascii_cols - 1) / 2;

    maze.cells.resize(maze.rows * maze.cols);

    double adjusted_cell_size{CELL_SIZE + (obstacle_size_adjustment * 2)};

    maze.cell_size = adjusted_cell_size;

    for (int r{0}; r < ascii_rows; r++) {
        for (int c{0}; c < ascii_cols; c++) {
            char ch{ascii.at(r).at(c)};
            geometry::Point center{ascii_to_world(r, c, obstacle_size_adjustment)};

            if (ch == '+') {
                maze.obstacles.push_back(create_post(center, obstacle_size_adjustment));
                size_t obstacle_index{maze.obstacles.size() - 1};
                attach_post_cells(maze, obstacle_index, r, c);
            } else if (ch == '|') {
                maze.obstacles.push_back(create_vertical_wall(center, obstacle_size_adjustment));
                size_t obstacle_index{maze.obstacles.size() - 1};
                attach_vertical_wall_cells(maze, obstacle_index, r, c);
            } else if (ch == '-') {
                maze.obstacles.push_back(create_horizontal_wall(center, obstacle_size_adjustment));
                size_t obstacle_index{maze.obstacles.size() - 1};
                attach_horizontal_wall_cells(maze, obstacle_index, r, c);
            } else if (ch == 'S') {
                int cell_r{r / 2};
                int cell_c{c / 2};

                maze.mouse_start = {(cell_c * adjusted_cell_size) + (adjusted_cell_size / 2),
                                    (cell_r * adjusted_cell_size) + (adjusted_cell_size / 2)};
            }
        }
    }

    return maze;
}

double compute_ray_distance_in_closed_space(const maze::Maze& maze, const geometry::Point& point,
                                            const geometry::Ray& ir_sensor)
{
    auto potential_distance{compute_ir_sensor_distance(maze, point, ir_sensor)};
    if (potential_distance.has_value()) {
        return *potential_distance;
    } else {
        throw std::runtime_error("No sensor reading in closed space");
    }
}

double compute_ray_distance_in_open_space(const maze::Maze& maze, const geometry::Point& point,
                                          const geometry::Ray& ir_sensor)
{
    auto potential_distance{compute_ir_sensor_distance(maze, point, ir_sensor)};
    if (potential_distance.has_value()) {
        return *potential_distance;
    } else {
        return 0;
    }
}

bool does_hitbox_collide_with_maze(const maze::Maze& maze,
                                   const geometry::RectangularHitbox& hitbox)
{
    auto rc{get_cell_from_point(maze, hitbox.center)};
    if (rc) {
        auto [r, c]{*rc};
        if (does_hitbox_collide_in_vicinity(maze, hitbox, r, c)) {
            return true;
        }
    } else {
        return true;
    }

    return false;
}

} /* maze namespace */

/*----------------------------------------------------------------------------*/
/*                             Private Definitions                            */
/*----------------------------------------------------------------------------*/
namespace
{

std::string validate_ascii_maze(const std::vector<std::string>& ascii)
{
    if (ascii.empty()) {
        return "ASCII maze is empty";
    }

    const size_t expected_cols{ascii.front().size()};
    bool found_start{false};

    for (size_t r{0}; r < ascii.size(); ++r) {
        const auto& row{ascii.at(r)};

        /* jagged check */
        if (row.size() != expected_cols) {
            return "ASCII maze is jagged (row " + std::to_string(r) + ")";
        }

        for (size_t c{0}; c < row.size(); ++c) {
            char ch{row.at(c)};

            /* character validation */
            if ((ch != ' ') && (ch != '+') && (ch != '|') && (ch != '-') && (ch != 'S')) {
                return "Invalid character '" + std::string(1, ch) + "' at (" + std::to_string(r)
                       + ", " + std::to_string(c) + ")";
            }

            /* start validation */
            if (ch == 'S') {
                if (found_start) {
                    return "Multiple 'S' start positions found";
                }
                found_start = true;
            }
        }
    }

    if (!found_start) {
        return "No 'S' start position found";
    }

    return ""; /* valid */
}

geometry::RectangularHitbox create_post(const geometry::Point& center, double size_adjustment)
{
    return geometry::RectangularHitbox{
        center,
        OFFICIAL_POST_SIZE + size_adjustment,
        OFFICIAL_POST_SIZE + size_adjustment
    };
}

geometry::RectangularHitbox create_vertical_wall(const geometry::Point& center,
                                                 double size_adjustment)
{
    return geometry::RectangularHitbox{
        center,
        OFFICIAL_WALL_WIDTH_SIZE + size_adjustment,
        OFFICIAL_WALL_LENGTH_SIZE + size_adjustment
    };
}

geometry::RectangularHitbox create_horizontal_wall(const geometry::Point& center,
                                                   double size_adjustment)
{
    return geometry::RectangularHitbox{
        center,
        OFFICIAL_WALL_LENGTH_SIZE + size_adjustment,
        OFFICIAL_WALL_WIDTH_SIZE + size_adjustment
    };
}

geometry::Point ascii_to_world(int r, int c, double size_adjustment)
{
    double post{OFFICIAL_POST_SIZE + size_adjustment};
    double wall{OFFICIAL_WALL_LENGTH_SIZE + size_adjustment};

    double pitch{post + wall};

    double x{(c / 2) * pitch};
    double y{(r / 2) * pitch};

    if ((c % 2) == 1) {
        x += pitch / 2; /* horizontal wall */
    }

    if ((r % 2) == 1) {
        y += pitch / 2; /* vertical wall */
    }

    return {x, y};
}

void attach_to_cell(Maze& maze, size_t obstacle_index, int row, int col)
{
    if (((row < 0) || (row >= maze.rows)) || ((col < 0) || (col >= maze.cols))) {
        return;
    }

    maze.cells.at(row * maze.cols + col).obstacles.push_back(obstacle_index);
}

void attach_vertical_wall_cells(Maze& maze, size_t obstacle_index, int r, int c)
{
    int cell_r{r / 2};
    int left_cell{(c / 2) - 1};
    int right_cell{(c / 2)};

    attach_to_cell(maze, obstacle_index, cell_r, left_cell);
    attach_to_cell(maze, obstacle_index, cell_r, right_cell);
}

void attach_horizontal_wall_cells(Maze& maze, size_t obstacle_index, int r, int c)
{
    int cell_c{c / 2};
    int bottom_cell{(r / 2) - 1};
    int top_cell{r / 2};

    attach_to_cell(maze, obstacle_index, bottom_cell, cell_c);
    attach_to_cell(maze, obstacle_index, top_cell, cell_c);
}

void attach_post_cells(Maze& maze, size_t obstacle_index, int r, int c)
{
    int base_r{(r / 2) - 1};
    int base_c{(c / 2) - 1};

    attach_to_cell(maze, obstacle_index, base_r, base_c);
    attach_to_cell(maze, obstacle_index, base_r + 1, base_c);
    attach_to_cell(maze, obstacle_index, base_r, base_c + 1);
    attach_to_cell(maze, obstacle_index, base_r + 1, base_c + 1);
}

std::optional<std::pair<int, int>> get_cell_from_point(const Maze& maze, const geometry::Point& p)
{
    const double cell{maze.cell_size};

    if ((cell <= 0.0) || (p.x < 0.0) || (p.y < 0.0)) {
        return std::nullopt;
    }

    const int col{static_cast<int>(p.x / cell)};
    const int row{static_cast<int>(p.y / cell)};

    if ((row < 0) || (row >= maze.rows)) {
        return std::nullopt;
    }

    if ((col < 0) || (col >= maze.cols)) {
        return std::nullopt;
    }

    return std::make_pair(row, col);
}

bool does_hitbox_collide_in_vicinity(const Maze& maze, const geometry::RectangularHitbox& hitbox,
                                     int row, int col)
{
    for (int dr{-1}; dr <= 1; dr++) {
        for (int dc{-1}; dc <= 1; dc++) {
            int r{row + dr};
            int c{col + dc};

            if (((r < 0) || (r >= maze.rows)) || ((c < 0) || (c >= maze.cols))) {
                continue;
            }

            const auto& cell{maze.get_cell(r, c)};

            for (size_t idx : cell.obstacles) {
                const auto& obstacle{maze.obstacles.at(idx)};

                if (geometry::do_hitboxes_overlap(hitbox, obstacle)) {
                    return true;
                }
            }
        }
    }

    return false;
}

std::optional<double> compute_ray_distance_in_cell(const Maze& maze, const geometry::Ray& ray,
                                                   int row, int col)
{
    if (((row < 0) || (row >= maze.rows)) || ((col < 0) || (col >= maze.cols))) {
        return std::nullopt;
    }

    const auto& cell{maze.get_cell(row, col)};

    std::optional<double> closest{std::nullopt};

    for (size_t idx : cell.obstacles) {
        const auto& obstacle{maze.obstacles.at(idx)};

        auto d{geometry::compute_ray_hitbox_distance(ray, obstacle)};

        if (!d) {
            continue;
        }

        if (!closest || (*d < *closest)) {
            closest = d;
        }
    }

    return closest;
}

std::optional<double> compute_ray_distance_in_vicinity(const Maze& maze, const geometry::Ray& ray,
                                                       int row, int col)
{
    std::optional<double> closest{std::nullopt};

    for (int dr{-1}; dr <= 1; dr++) {
        for (int dc{-1}; dc <= 1; dc++) {
            auto d{compute_ray_distance_in_cell(maze, ray, row + dr, col + dc)};

            if (!d) {
                continue;
            }

            if (!closest || (*d < *closest)) {
                closest = d;
            }
        }
    }

    return closest;
}

std::optional<double> compute_ir_sensor_distance(const maze::Maze& maze,
                                                 const geometry::Point& point,
                                                 const geometry::Ray& ir_sensor)
{
    std::optional<double> distance{std::nullopt};

    auto potential_rc{get_cell_from_point(maze, point)};
    if (potential_rc) {
        auto [r, c]{*potential_rc};
        auto potential_distance{compute_ray_distance_in_vicinity(maze, ir_sensor, r, c)};
        if (potential_distance.has_value()) {
            distance = *potential_distance;
        }
    }

    return distance;
}

} /* unnamed namespace */
