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

struct ControlConfig {
    uint32_t reading_threshold;
    double reading_start_offset; /* 0-0.9 */
    uint32_t slope_threshold;
};

struct EnvironmentConfig {
    double maze_size_scale;
    double ir_reading_scale;
    double mouse_angle;
    double horizontal_position_variance;
    double vertical_position_variance;
    int total_steps;
};

struct Config {
    ControlConfig ctrl_cfg;
    EnvironmentConfig env_cfg;
};

struct Result {
    bool wall_absent_correct;
    bool wall_present_correct;
};

} /* side_wall_detection namespace */

/*----------------------------------------------------------------------------*/
/*                             Public Declarations                            */
/*----------------------------------------------------------------------------*/
namespace side_wall_detection
{

void reset_all_config_bounds(void);
void set_ctr_config_bounds(const ControlConfig& lower, const ControlConfig& upper);
void set_env_config_bounds(const EnvironmentConfig& lower, const EnvironmentConfig& upper);
ControlConfig decode_control(const std::vector<double>& x);
std::vector<double> encode_control(const ControlConfig& cfg);
std::pair<std::vector<double>, std::vector<double>> get_control_bounds(void);
EnvironmentConfig generate_random_environment(void);

void enable_visualization(const std::string& foldername);
void disable_visualization(void);

std::string config_to_string(const Config& cfg);
Result run_simulation(const Config& cfg);

} /* side_wall_detection namespace */

#endif /* SIDE_WALL_DETECTION_HPP_ */
