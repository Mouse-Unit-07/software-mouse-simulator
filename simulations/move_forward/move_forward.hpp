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

struct Config {
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

    uint8_t motor_speed;
    int32_t kp;
    int32_t kd;
    int32_t pid_shift;
};

class ConfigSweeper {
public:
    std::vector<double> dt;
    std::vector<double> motor_speed_scale;
    std::vector<double> motor1_variance;
    std::vector<double> motor2_variance;
    std::vector<double> slip_factor;
    std::vector<double> wheel_circumference_scale;
    std::vector<double> wheel_base_scale;
    std::vector<double> maze_size_scale;
    std::vector<double> ir_reading_scale;
    std::vector<double> mouse_angle;
    std::vector<double> horizontal_position_variance;
    std::vector<double> vertical_position_variance;

    std::vector<uint8_t> motor_speed;
    std::vector<int32_t> kp;
    std::vector<int32_t> kd;
    std::vector<int32_t> pid_shift;

    bool next(void);
    Config value(void) const;

private:
    simulation_common::CommonConfigSweeper sweeper;
    bool initialized_{false};
};

} /* move_forward namespace */

/*----------------------------------------------------------------------------*/
/*                             Public Declarations                            */
/*----------------------------------------------------------------------------*/
namespace move_forward
{

} /* move_forward namespace */

#endif /* MOVE_FORWARD_HPP_ */
