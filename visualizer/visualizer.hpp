/*-------------------------------- FILE INFO ---------------------------------*/
/* Filename           : visualizer.hpp                                        */
/*                                                                            */
/* Interface to a micromouse simulation visualizer                            */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#ifndef VISUALIZER_HPP_
#define VISUALIZER_HPP_

/*----------------------------------------------------------------------------*/
/*                             Public Definitions                             */
/*----------------------------------------------------------------------------*/
/* none */

/*----------------------------------------------------------------------------*/
/*                             Public Declarations                            */
/*----------------------------------------------------------------------------*/
namespace visualizer
{

class Visualizer
{
public:
    Visualizer() = default;

    void draw_maze(float cell_size_pixels, const maze::Maze& maze);
    void draw_mouse_on_maze(const mouse::Mouse& mouse);
    void draw_ir_1_sensor_beam(const mouse::Mouse& mouse, double length_mm);
    void draw_ir_2_sensor_beam(const mouse::Mouse& mouse, double length_mm);
    void draw_ir_3_sensor_beam(const mouse::Mouse& mouse, double length_mm);
    void draw_ir_4_sensor_beam(const mouse::Mouse& mouse, double length_mm);
    void save_to_image_file(const std::string& filename);

private:
    float cell_size;
    sf::RenderTexture texture;

    sf::Vector2f world_to_screen(const geometry::Point& p) const;
    sf::RectangleShape make_rectangle(const geometry::RectangularHitbox& hitbox) const;
    
    void draw_cells(const maze::Maze& maze);
    void draw_obstacles(const maze::Maze& maze);
    void draw_mouse_start(const maze::Maze& maze);
    void draw_ray(const geometry::Ray& ray);
    void draw_ray_beam(const geometry::Ray& ray, double length_mm);
};

} /* visualizer namespace */

#endif /* VISUALIZER_HPP_ */
