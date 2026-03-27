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
    double total_steps;

    double reading_threshold;
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

Config build_config(const std::vector<double>& v);

} /* wall_detection namespace */

#endif /* WALL_DETECTION_HPP_ */
