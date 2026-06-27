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
    uint32_t single_wall_target{};
    uint8_t motor_speed{};
    int32_t kp_velocity{};
    int32_t kd_velocity{};
    int32_t kp_angle{};
    int32_t kd_angle{};
    int32_t pid_scale{};
    int32_t kp_ir{};
    int32_t kd_ir{};
};

struct EnvironmentConfig {
    double dt{};
    double motor_speed_scale{};
    double motor1_variance{};
    double motor2_variance{};
    double slip_factor{};
    double wheel_circumference_scale{};
    double wheel_base_scale{};
    double maze_post_size_scale{};
    double maze_wall_size_scale{};
    double ir_reading_scale{};
    double mouse_angle{};
    double horizontal_position_variance{};
    double vertical_position_variance{};
};

struct Config {
    ControlConfig ctrl_cfg{};
    EnvironmentConfig env_cfg{};
};

struct Result {
    double total_time{0.0};
    double final_angle_error{0.0};
    double final_horizontal_translation{0.0};
    double final_vertical_translation{0.0};
    bool collision{false};
    bool timeout{false};
};

enum class WallMode
{
    NO_WALLS,
    LEFT_WALL_ONLY,
    RIGHT_WALL_ONLY,
    BOTH_WALLS
};

const std::vector<WallMode> WALL_MODES{WallMode::NO_WALLS, WallMode::LEFT_WALL_ONLY,
                                       WallMode::BOTH_WALLS};

} /* move_forward namespace */

/*----------------------------------------------------------------------------*/
/*                             Public Declarations                            */
/*----------------------------------------------------------------------------*/
namespace move_forward
{

void reset_all_config_bounds(void);
void set_ctr_config_bounds(const ControlConfig &lower, const ControlConfig &upper);
void set_env_config_bounds(const EnvironmentConfig &lower, const EnvironmentConfig &upper);
ControlConfig decode_control(const std::vector<double> &x);
std::vector<double> encode_control(const ControlConfig &cfg);
std::pair<std::vector<double>, std::vector<double>> get_control_bounds(void);
EnvironmentConfig generate_random_environment(void);

void enable_visualization(const std::string &foldername);
void disable_visualization(void);
std::string config_to_string(const Config &cfg);
std::string wall_mode_to_string(WallMode mode);

Result run_simulation(const Config &cfg, enum WallMode mode);

} /* move_forward namespace */

#endif /* MOVE_FORWARD_HPP_ */
