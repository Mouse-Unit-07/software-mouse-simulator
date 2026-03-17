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
    void save_to_image_file(const std::string& filename);

private:
    float cell_size;
    sf::RenderTexture texture;

    sf::Vector2f world_to_screen(const geometry::Point& p, const maze::Maze& maze) const;
    sf::RectangleShape make_rectangle(const geometry::RectangularHitbox& hitbox, const maze::Maze& maze) const;
    
    void draw_cells(sf::RenderTarget& target, const maze::Maze& maze) const;
    void draw_obstacles(sf::RenderTarget& target, const maze::Maze& maze) const;
    void draw_mouse_start(sf::RenderTarget& target, const maze::Maze& maze) const;
};

} /* visualizer namespace */

#endif /* VISUALIZER_HPP_ */
