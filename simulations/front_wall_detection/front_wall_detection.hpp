/*-------------------------------- FILE INFO ---------------------------------*/
/* Filename           : front_wall_detection.hpp                              */
/*                                                                            */
/* Interface to functions to run micromouse front wall detection simulation   */
/* and associated config and results analysis helpers                         */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#ifndef FRONT_WALL_DETECTION_HPP_
#define FRONT_WALL_DETECTION_HPP_

/*----------------------------------------------------------------------------*/
/*                             Public Definitions                             */
/*----------------------------------------------------------------------------*/
namespace front_wall_detection
{

struct ControlConfig {
    uint32_t reading_threshold;
};

struct EnvironmentConfig {
    double ir_reading_scale;
    double mouse_angle;
    double horizontal_position_variance;
    double vertical_position_variance;
};

struct Config {
    ControlConfig ctrl_cfg;
    EnvironmentConfig env_cfg;
};

struct Result {
    bool identified_absent_wall;
    bool identified_present_wall;
};

} /* front_wall_detection namespace */

/*----------------------------------------------------------------------------*/
/*                             Public Declarations                            */
/*----------------------------------------------------------------------------*/
namespace front_wall_detection
{

void enable_visualization(void);
void disable_visualization(void);
std::string config_to_string(const Config& cfg);

Result run_simulation(const Config& cfg);

} /* front_wall_detection namespace */

#endif /* FRONT_WALL_DETECTION_HPP_ */
