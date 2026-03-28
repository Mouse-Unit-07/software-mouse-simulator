/*-------------------------------- FILE INFO ---------------------------------*/
/* Filename           : wall_detection.hpp                                    */
/*                                                                            */
/* Interface to functions to run micromouse wall detection simulation and     */ 
/* associated config and results analysis helpers                             */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#ifndef WALL_DETECTION_HPP_
#define WALL_DETECTION_HPP_

/*----------------------------------------------------------------------------*/
/*                             Public Definitions                             */
/*----------------------------------------------------------------------------*/
namespace wall_detection
{

struct Config
{
    double maze_size_scale;
    double ir_reading_scale;
    double mouse_angle;
    double horizontal_position_variance;
    double vertical_position_variance;
    int total_steps;

    uint32_t reading_threshold;
};

struct Result
{
    std::vector<bool> correct_detection_at_step;
};

} /* wall_detection namespace */

/*----------------------------------------------------------------------------*/
/*                             Public Declarations                            */
/*----------------------------------------------------------------------------*/
namespace wall_detection
{

void enable_visualization(void);
void disable_visualization(void);

Config build_config(const std::vector<double>& v);
std::string config_to_string(const Config& cfg);
Result run_simulation(const Config& cfg);

} /* wall_detection namespace */

#endif /* WALL_DETECTION_HPP_ */
