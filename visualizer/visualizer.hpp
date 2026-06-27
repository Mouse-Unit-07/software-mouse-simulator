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
namespace visualizer
{

class Visualizer {
public:
    Visualizer();
    ~Visualizer();

    void draw_maze(double cell_size_pixels, const maze::Maze &maze);
    void draw_mouse_on_maze(const mouse::Mouse &mouse);

    void draw_ir_1_sensor_beam(const mouse::Mouse &mouse, double length_mm);
    void draw_ir_2_sensor_beam(const mouse::Mouse &mouse, double length_mm);
    void draw_ir_3_sensor_beam(const mouse::Mouse &mouse, double length_mm);
    void draw_ir_4_sensor_beam(const mouse::Mouse &mouse, double length_mm);

    void reset_beam_color(void);
    void change_beam_color_to_red(void);
    void reset_mouse_color(void);
    void change_mouse_color_to_green(void);
    void change_mouse_color_to_blue(void);

    void save_to_image_file(const std::string &filename);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} /* visualizer namespace */

/*----------------------------------------------------------------------------*/
/*                             Public Declarations                            */
/*----------------------------------------------------------------------------*/
/* none */

#endif /* VISUALIZER_HPP_ */
