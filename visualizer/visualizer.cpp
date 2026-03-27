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
#include <optional>
#include <utility>
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

void Visualizer::draw_maze(float cell_size_pixels, const maze::Maze& maze)
{
    cell_size = cell_size_pixels;
    int width  {static_cast<int>(maze.cols * cell_size)};
    int height {static_cast<int>(maze.rows * cell_size)};

    texture.create(width, height);
    texture.clear(sf::Color::Black);
    
    draw_cells(maze);
    draw_obstacles(maze);
    draw_mouse_start(maze);
}

void Visualizer::draw_mouse_on_maze(const mouse::Mouse& mouse)
{
    auto rect {make_rectangle(mouse.hitbox, sf::Color::Red)};
    texture.draw(rect);

    draw_ray(mouse.ir_1_sensor);
    draw_ray(mouse.ir_2_sensor);
    draw_ray(mouse.ir_3_sensor);
    draw_ray(mouse.ir_4_sensor);
}

void Visualizer::draw_ir_1_sensor_beam(const mouse::Mouse& mouse, double length_mm)
{
    draw_ray_beam(mouse.ir_1_sensor, length_mm);
}

void Visualizer::draw_ir_2_sensor_beam(const mouse::Mouse& mouse, double length_mm)
{
    draw_ray_beam(mouse.ir_2_sensor, length_mm);
}

void Visualizer::draw_ir_3_sensor_beam(const mouse::Mouse& mouse, double length_mm)
{
    draw_ray_beam(mouse.ir_3_sensor, length_mm);
}

void Visualizer::draw_ir_4_sensor_beam(const mouse::Mouse& mouse, double length_mm)
{
    draw_ray_beam(mouse.ir_4_sensor, length_mm);
}

void Visualizer::save_to_image_file(const std::string& filename)
{
    texture.display();
    sf::Image image {texture.getTexture().copyToImage()};
    image.saveToFile(filename);
}

} /* visualizer namespace */

/*----------------------------------------------------------------------------*/
/*                             Private Definitions                            */
/*----------------------------------------------------------------------------*/
namespace visualizer
{

sf::Vector2f Visualizer::world_to_screen(const geometry::Point& p) const
{
    float scale {static_cast<float>(cell_size / maze::CELL_SIZE)};

    return {
        static_cast<float>(p.x * scale),
        static_cast<float>(p.y * scale)
    };
}

sf::VertexArray Visualizer::make_rectangle(const geometry::RectangularHitbox& hitbox, sf::Color color) const
{
    sf::VertexArray outline(sf::LineStrip, 5);

    outline[0].position = world_to_screen(hitbox.top_right);
    outline[1].position = world_to_screen(hitbox.top_left);
    outline[2].position = world_to_screen(hitbox.bottom_left);
    outline[3].position = world_to_screen(hitbox.bottom_right);
    outline[4].position = world_to_screen(hitbox.top_right);

    for (int i {0}; i < 5; ++i) {
        outline[i].color = color;
    }

    return outline;
}

void Visualizer::draw_cells(const maze::Maze& maze)
{
    sf::RectangleShape cell;
    cell.setSize({cell_size, cell_size});
    cell.setFillColor(sf::Color::Transparent);
    cell.setOutlineColor(sf::Color(60, 60, 60));
    cell.setOutlineThickness(1.0f);

    for (int r {0}; r < maze.rows; ++r) {
        for (int c {0}; c < maze.cols; ++c) {
            cell.setPosition(c * cell_size, r * cell_size);
            texture.draw(cell);
        }
    }
}

void Visualizer::draw_obstacles(const maze::Maze& maze)
{
    for (const auto& obstacle : maze.obstacles) {
        auto rect {make_rectangle(obstacle, sf::Color::White)};
        texture.draw(rect);
    }
}

void Visualizer::draw_mouse_start(const maze::Maze& maze)
{
    sf::CircleShape marker;
    marker.setRadius(cell_size * 0.05f);
    marker.setFillColor(sf::Color::Green);

    auto pos {world_to_screen(maze.mouse_start)};
    marker.setPosition(pos.x - marker.getRadius(), pos.y - marker.getRadius());

    texture.draw(marker);
}

void Visualizer::draw_ray(const geometry::Ray& ray)
{
    float scale {static_cast<float>(cell_size / maze::CELL_SIZE)};
    double ray_length {scale * 20.0f};

    sf::Vector2f origin {world_to_screen(ray.origin)};
    geometry::Point ray_end {ray.origin.x + (ray.direction.x * ray_length), ray.origin.y + (ray.direction.y * ray_length)};
    sf::Vector2f end {world_to_screen(ray_end)};

    sf::Vertex line[]
    {
        sf::Vertex(origin, sf::Color::Yellow),
        sf::Vertex(end,    sf::Color::Yellow)
    };

    texture.draw(line, 2, sf::Lines);
}

void Visualizer::draw_ray_beam(const geometry::Ray& ray, double length_mm)
{
    geometry::Point end_world {
        ray.origin.x + ray.direction.x * length_mm,
        ray.origin.y + ray.direction.y * length_mm
    };

    sf::Vector2f origin {world_to_screen(ray.origin)};
    sf::Vector2f end {world_to_screen(end_world)};

    sf::Vertex line[]
    {
        sf::Vertex(origin, sf::Color::Cyan),
        sf::Vertex(end,    sf::Color::Cyan)
    };

    texture.draw(line, 2, sf::Lines);
}

} /* visualizer namespace */
