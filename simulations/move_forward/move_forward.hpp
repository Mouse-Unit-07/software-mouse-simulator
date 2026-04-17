/*-------------------------------- FILE INFO ---------------------------------*/
/* Filename           : move_forward.hpp                                      */
/*                                                                            */
/* Interface to functions to run micromouse move_forward simulation and       */
/* associated config and results analysis helpers                             */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#ifndef MOVE_FORWARD_HPP_
#define MOVE_FORWARD_HPP_

/*----------------------------------------------------------------------------*/
/*                             Public Definitions                             */
/*----------------------------------------------------------------------------*/
namespace move_forward
{

struct ControlConfig {
    uint32_t single_wall_target;
    uint8_t motor_speed;
    int32_t kp;
    int32_t kd;
    int32_t pid_scale;
    int32_t kp_ir;
    int32_t kd_ir;
};

struct EnvironmentConfig {
    double dt;
    double motor_speed_scale;
    double motor1_variance;
    double motor2_variance;
    double slip_factor;
    double wheel_circumference_scale;
    double wheel_base_scale;
    double maze_size_scale;
    double ir_reading_scale;
    double mouse_angle;
    double horizontal_position_variance;
    double vertical_position_variance;
};

struct Config {
    ControlConfig ctrl_cfg;
    EnvironmentConfig env_cfg;
};

struct SingleCaseResult {
    double total_time{0.0};
    double total_angle_error{0.0};
    double total_horizontal_translation{0.0};
    double final_vertical_translation{0.0};
    bool collision{false};
    bool timeout{false};
};

struct Result {
    SingleCaseResult no_wall;
    SingleCaseResult one_wall;
    SingleCaseResult two_wall;
};

enum wall_mode
{
    NO_WALLS,
    LEFT_WALL_ONLY,
    RIGHT_WALL_ONLY,
    BOTH_WALLS
};

} /* move_forward namespace */

/*----------------------------------------------------------------------------*/
/*                             Public Declarations                            */
/*----------------------------------------------------------------------------*/
namespace move_forward
{

ControlConfig decode_control(const std::vector<double>& x);
std::vector<double> encode_control(const ControlConfig& cfg);
std::pair<std::vector<double>, std::vector<double>> get_control_bounds(void);
EnvironmentConfig generate_random_environment(void);

void enable_visualization(const std::string& foldername);
void disable_visualization(void);
std::string config_to_string(const Config& cfg);

Result run_simulation(const Config& cfg);

} /* move_forward namespace */

#endif /* MOVE_FORWARD_HPP_ */
