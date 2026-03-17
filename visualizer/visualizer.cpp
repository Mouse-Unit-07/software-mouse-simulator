/*-------------------------------- FILE INFO ---------------------------------*/
/* Filename           : visualizer.cpp                                        */
/*                                                                            */
/* Implementation of a micromouse simulation visualizer                       */
/*                                                                            */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*                               Include Files                                */
/*----------------------------------------------------------------------------*/
extern "C"
{

}

#include <cmath>
#include <vector>
#include <string>
#include <SFML/Graphics.hpp>
#include "point.hpp"
#include "ray.hpp"
#include "rectangular_hitbox.hpp"
#include "mouse.hpp"
#include "maze.hpp"
#include "visualizer.hpp"

/*----------------------------------------------------------------------------*/
/*                            Private Declarations                            */
/*----------------------------------------------------------------------------*/
/* none */

/*----------------------------------------------------------------------------*/
/*                               Private Globals                              */
/*----------------------------------------------------------------------------*/
/* none */

/*----------------------------------------------------------------------------*/
/*                             Public Definitions                             */
/*----------------------------------------------------------------------------*/
namespace visualizer
{

Visualizer::Visualizer(float cell_size_pixels)
    : cell_size{cell_size_pixels}
{
    /* no additional logic */
}

void Visualizer::draw(sf::RenderTarget& target, const maze::Maze& maze) const
{
    draw_cells(target, maze);
    draw_obstacles(target, maze);
    draw_mouse_start(target, maze);
}

} /* visualizer namespace */

/*----------------------------------------------------------------------------*/
/*                             Private Definitions                            */
/*----------------------------------------------------------------------------*/
namespace visualizer
{

sf::Vector2f Visualizer::world_to_screen(const geometry::Point& p, const maze::Maze& maze) const
{
    float scale {static_cast<float>(cell_size / maze::CELL_SIZE)};

    return {
        static_cast<float>(p.x * scale),
        static_cast<float>(p.y * scale)
    };
}

sf::RectangleShape Visualizer::make_rectangle(const geometry::RectangularHitbox& hitbox, const maze::Maze& maze) const
{
    float scale {static_cast<float>(cell_size / maze::CELL_SIZE)};

    float width  {static_cast<float>(hitbox.horizontal_size * scale)};
    float height {static_cast<float>(hitbox.vertical_size * scale)};

    sf::RectangleShape rect({width, height});

    auto pos {world_to_screen(hitbox.center, maze)};

    rect.setPosition(
        pos.x - width  / 2.0f,
        pos.y - height / 2.0f
    );

    return rect;
}

void Visualizer::draw_cells(sf::RenderTarget& target, const maze::Maze& maze) const
{
    sf::RectangleShape cell;
    cell.setSize({cell_size, cell_size});
    cell.setFillColor(sf::Color::Transparent);
    cell.setOutlineColor(sf::Color(60, 60, 60));
    cell.setOutlineThickness(1.0f);

    for (int r {0}; r < maze.rows; ++r)
    {
        for (int c {0}; c < maze.cols; ++c)
        {
            cell.setPosition(
                c * cell_size,
                r * cell_size
            );

            target.draw(cell);
        }
    }
}

void Visualizer::draw_obstacles(sf::RenderTarget& target, const maze::Maze& maze) const
{
    for (const auto& obstacle : maze.obstacles)
    {
        auto rect {make_rectangle(obstacle, maze)};
        rect.setFillColor(sf::Color::White);
        target.draw(rect);
    }
}

void Visualizer::draw_mouse_start(sf::RenderTarget& target, const maze::Maze& maze) const
{
    sf::CircleShape marker;
    marker.setRadius(cell_size * 0.05f);
    marker.setFillColor(sf::Color::Green);

    auto pos {world_to_screen(maze.mouse_start, maze)};
    marker.setPosition(pos.x - marker.getRadius(), pos.y - marker.getRadius());

    target.draw(marker);
}

} /* visualizer namespace */
