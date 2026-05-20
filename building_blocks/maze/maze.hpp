/*-------------------------------- FILE INFO ---------------------------------*/
/* Filename           : optimizer.hpp                                         */
/*                                                                            */
/* Interface to maze building logic for micromouse simulations                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#ifndef MAZE_HPP_
#define MAZE_HPP_

/*----------------------------------------------------------------------------*/
/*                             Public Definitions                             */
/*----------------------------------------------------------------------------*/
namespace maze
{

constexpr double OFFICIAL_POST_SIZE{12.07};
constexpr double OFFICIAL_WALL_LENGTH_SIZE{166.37};
constexpr double OFFICIAL_WALL_WIDTH_SIZE{12.07};

constexpr double CELL_SIZE{OFFICIAL_WALL_LENGTH_SIZE + OFFICIAL_POST_SIZE};

struct Cell {
    std::vector<size_t> obstacles{};
};

class Maze {
public:
    int rows{0};
    int cols{0};
    double cell_size{0.0};
    std::vector<geometry::RectangularHitbox> obstacles{};
    std::vector<Cell> cells{};
    geometry::Point mouse_start{};

    const Cell& get_cell(int row, int col) const;
};

} /* maze namespace */

/*----------------------------------------------------------------------------*/
/*                             Public Declarations                            */
/*----------------------------------------------------------------------------*/
namespace maze
{

Maze build_maze_from_ascii(const std::vector<std::string>& ascii, double post_size_scale);

double compute_ray_distance_in_closed_space(const maze::Maze& maze, const geometry::Point& point,
                                            const geometry::Ray& ir_sensor);
double compute_ray_distance_in_open_space(const maze::Maze& maze, const geometry::Point& point,
                                          const geometry::Ray& ir_sensor);

bool does_hitbox_collide_with_maze(const maze::Maze& maze,
                                   const geometry::RectangularHitbox& hitbox);

} /* maze namespace */

#endif /* MAZE_HPP_ */
