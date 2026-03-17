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
    Visualizer(float cell_size_pixels);

    void draw(sf::RenderTarget& target, const maze::Maze& maze) const;

private:
    float cell_size;

    sf::Vector2f world_to_screen(const geometry::Point& p, const maze::Maze& maze) const;
    sf::RectangleShape make_rectangle(const geometry::RectangularHitbox& hitbox, const maze::Maze& maze) const;
    
    void draw_cells(sf::RenderTarget& target, const maze::Maze& maze) const;
    void draw_obstacles(sf::RenderTarget& target, const maze::Maze& maze) const;
    void draw_mouse_start(sf::RenderTarget& target, const maze::Maze& maze) const;
};

inline void render_maze_to_image(const maze::Maze& maze, float cell_size, const std::string& filename)
{
    int width  = static_cast<int>(maze.cols * cell_size);
    int height = static_cast<int>(maze.rows * cell_size);

    sf::RenderTexture texture;
    texture.create(width, height);

    Visualizer visualizer(cell_size);

    texture.clear(sf::Color::Black);
    visualizer.draw(texture, maze);
    texture.display();

    sf::Image image = texture.getTexture().copyToImage();
    image.saveToFile(filename);
}

} /* visualizer namespace */

#endif /* VISUALIZER_HPP_ */
