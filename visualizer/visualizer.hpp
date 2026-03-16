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

    void draw_cells(sf::RenderTarget& target, const maze::Maze& maze) const;
    void draw_obstacles(sf::RenderTarget& target, const maze::Maze& maze) const;
    void draw_mouse_start(sf::RenderTarget& target, const maze::Maze& maze) const;

    sf::Vector2f world_to_screen(const geometry::Point& p, const maze::Maze& maze) const;
    sf::RectangleShape make_rectangle(const geometry::RectangularHitbox& hitbox, const maze::Maze& maze) const;
};

inline void run_visual_maze_test(const maze::Maze& maze, float cell_size = 100.0f)
{
    sf::RenderWindow window(
        sf::VideoMode(800, 800),
        "Micromouse Visual Test"
    );

    Visualizer visualizer(cell_size);

    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        window.clear(sf::Color::Black);

        visualizer.draw(window, maze);

        window.display();
    }
}

} /* visualizer namespace */

#endif /* VISUALIZER_HPP_ */
