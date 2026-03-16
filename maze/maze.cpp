/*-------------------------------- FILE INFO ---------------------------------*/
/* Filename           : maze.cpp                                              */
/*                                                                            */
/* Implementation for maze building logic for micromouse simulations          */
/*                                                                            */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*                               Include Files                                */
/*----------------------------------------------------------------------------*/
extern "C"
{

}

#include <vector>
#include <string>
#include "point.hpp"
#include "rectangular_hitbox.hpp"
#include "maze.hpp"

/*----------------------------------------------------------------------------*/
/*                            Private Declarations                            */
/*----------------------------------------------------------------------------*/
namespace
{

geometry::RectangularHitbox create_post(const geometry::Point& center, double size_adjustment);
geometry::RectangularHitbox create_vertical_wall(const geometry::Point& center, double size_adjustment);
geometry::RectangularHitbox create_horizontal_wall(const geometry::Point& center, double size_adjustment);

geometry::Point ascii_to_world(int r, int c, double cell_size);
void attach_to_cell(maze::Maze& maze, const geometry::RectangularHitbox& obstacle,
        int row, int col);
void attach_vertical_wall_cells(maze::Maze& maze, const geometry::RectangularHitbox& wall,
        int r, int c);
void attach_horizontal_wall_cells(maze::Maze& maze, const geometry::RectangularHitbox& wall,
        int r, int c);
void attach_post_cells(maze::Maze& maze, const geometry::RectangularHitbox& post,
        int r, int c);

}

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
    return cells[row * cols + col];
}

Maze build_from_ascii(const std::vector<std::string>& ascii, double obstacle_size_adjustment)
{
    Maze maze;

    int ascii_rows = ascii.size();
    int ascii_cols = ascii[0].size();

    maze.rows = (ascii_rows - 1) / 2;
    maze.cols = (ascii_cols - 1) / 2;

    maze.cells.resize(maze.rows * maze.cols);

    double adjusted_cell_size = CELL_SIZE + (obstacle_size_adjustment * 2);

    for (int r = 0; r < ascii_rows; ++r)
    {
        for (int c = 0; c < ascii_cols; ++c)
        {
            char ch = ascii[r][c];

            geometry::Point center = ascii_to_world(r, c, adjusted_cell_size);

            if (ch == '+')
            {
                maze.obstacles.push_back(create_post(center, obstacle_size_adjustment));
                attach_post_cells(maze, maze.obstacles.back(), r, c);
            }

            else if (ch == '|')
            {
                maze.obstacles.push_back(create_vertical_wall(center, obstacle_size_adjustment));
                attach_vertical_wall_cells(maze, maze.obstacles.back(), r, c);
            }

            else if (ch == '-')
            {
                maze.obstacles.push_back(create_horizontal_wall(center, obstacle_size_adjustment));
                attach_horizontal_wall_cells(maze, maze.obstacles.back(), r, c);
            }

            else if (ch == 'S')
            {
                int cell_r = r / 2;
                int cell_c = c / 2;

                maze.mouse_start =
                {
                    cell_c * adjusted_cell_size + adjusted_cell_size / 2,
                    cell_r * adjusted_cell_size + adjusted_cell_size / 2
                };
            }
        }
    }

    return maze;
}

} /* maze namespace */

/*----------------------------------------------------------------------------*/
/*                             Private Definitions                            */
/*----------------------------------------------------------------------------*/
namespace
{

geometry::RectangularHitbox create_post(const geometry::Point& center, double size_adjustment)
{
    return geometry::RectangularHitbox{
        center,
        maze::OFFICIAL_POST_SIZE + size_adjustment,
        maze::OFFICIAL_POST_SIZE + size_adjustment
    };
}

geometry::RectangularHitbox create_vertical_wall(const geometry::Point& center, double size_adjustment)
{
    return geometry::RectangularHitbox{
        center,
        maze::OFFICIAL_WALL_WIDTH_SIZE + size_adjustment,
        maze::OFFICIAL_WALL_LENGTH_SIZE + size_adjustment
    };
}

geometry::RectangularHitbox create_horizontal_wall(const geometry::Point& center, double size_adjustment)
{
    return geometry::RectangularHitbox{
        center,
        maze::OFFICIAL_WALL_LENGTH_SIZE + size_adjustment,
        maze::OFFICIAL_WALL_WIDTH_SIZE + size_adjustment
    };
}

geometry::Point ascii_to_world(int r, int c, double cell_size)
{
    double x = (c / 2.0) * cell_size;
    double y = (r / 2.0) * cell_size;

    return {x, y};
}

void attach_to_cell(maze::Maze& maze, const geometry::RectangularHitbox& obstacle,
        int row, int col)
{
    if (row < 0 || row >= maze.rows) return;
    if (col < 0 || col >= maze.cols) return;

    maze.cells[row * maze.cols + col]
        .obstacles.push_back(&obstacle);
}

void attach_vertical_wall_cells(maze::Maze& maze, const geometry::RectangularHitbox& wall,
        int r, int c)
{
    int cell_r = r / 2;
    int left_cell  = (c / 2) - 1;
    int right_cell = (c / 2);

    attach_to_cell(maze, wall, cell_r, left_cell);
    attach_to_cell(maze, wall, cell_r, right_cell);
}

void attach_horizontal_wall_cells(maze::Maze& maze, const geometry::RectangularHitbox& wall,
        int r, int c)
{
    int cell_c = c / 2;
    int bottom_cell = (r / 2) - 1;
    int top_cell    = (r / 2);

    attach_to_cell(maze, wall, bottom_cell, cell_c);
    attach_to_cell(maze, wall, top_cell, cell_c);
}

void attach_post_cells(maze::Maze& maze, const geometry::RectangularHitbox& post,
        int r, int c)
{
    int base_r = (r / 2) - 1;
    int base_c = (c / 2) - 1;

    attach_to_cell(maze, post, base_r,     base_c);
    attach_to_cell(maze, post, base_r + 1, base_c);
    attach_to_cell(maze, post, base_r,     base_c + 1);
    attach_to_cell(maze, post, base_r + 1, base_c + 1);
}

}
