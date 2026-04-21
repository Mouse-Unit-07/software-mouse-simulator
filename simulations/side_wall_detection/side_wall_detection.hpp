/*-------------------------------- FILE INFO ---------------------------------*/
/* Filename           : side_wall_detection.hpp                               */
/*                                                                            */
/* Interface to functions to run micromouse side wall detection simulation    */
/* and associated config and results analysis helpers                         */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#ifndef SIDE_WALL_DETECTION_HPP_
#define SIDE_WALL_DETECTION_HPP_

/*----------------------------------------------------------------------------*/
/*                             Public Definitions                             */
/*----------------------------------------------------------------------------*/
namespace side_wall_detection
{

struct Config {
    double maze_size_scale;
    double ir_reading_scale;
    double mouse_angle;
    double horizontal_position_variance;
    double vertical_position_variance;
    int total_steps;

    uint32_t reading_threshold;
};

struct Result {
    std::vector<bool> wall_absent_at_step;
    std::vector<bool> wall_present_at_step;
};

} /* side_wall_detection namespace */

/*----------------------------------------------------------------------------*/
/*                             Public Declarations                            */
/*----------------------------------------------------------------------------*/
namespace side_wall_detection
{

void enable_visualization(void);
void disable_visualization(void);

std::string config_to_string(const Config& cfg);
Result run_simulation(const Config& cfg);

} /* side_wall_detection namespace */

#endif /* SIDE_WALL_DETECTION_HPP_ */
