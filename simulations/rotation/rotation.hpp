/*-------------------------------- FILE INFO ---------------------------------*/
/* Filename           : rotation.hpp                                          */
/*                                                                            */
/* Interface to functions to run micromouse rotation simulation and           */
/* associated config and results analysis helpers                             */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#ifndef ROTATION_HPP_
#define ROTATION_HPP_

/*----------------------------------------------------------------------------*/
/*                             Public Definitions                             */
/*----------------------------------------------------------------------------*/
namespace rotation
{

struct ControlConfig {
    uint8_t motor_speed;
    int32_t kp_velocity;
    int32_t kd_velocity;
    int32_t kp_angle;
    int32_t kd_angle;
    int32_t pid_scale;
};

struct EnvironmentConfig {
    double dt;
    double motor_speed_scale;
    double motor1_variance;
    double motor2_variance;
    double slip_factor;
    double wheel_circumference_scale;
    double wheel_base_scale;
    double rotation_angle;
};

struct Config {
    ControlConfig ctrl_cfg;
    EnvironmentConfig env_cfg;
};

struct Result {
    double total_time{0.0};
    double final_angle_error{0.0};
    double total_translation{0.0};
    bool collision{false};
    bool timeout{false};
};

} /* rotation namespace */

/*----------------------------------------------------------------------------*/
/*                             Public Declarations                            */
/*----------------------------------------------------------------------------*/
namespace rotation
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

} /* rotation namespace */

#endif /* ROTATION_HPP_ */
