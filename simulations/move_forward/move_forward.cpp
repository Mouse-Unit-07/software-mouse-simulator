/*-------------------------------- FILE INFO ---------------------------------*/
/* Filename           : move_forward.cpp                                      */
/*                                                                            */
/* Implementation for micromouse move_forward simulation and associated       */
/* config and results analysis helpers                                        */
/*                                                                            */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*                               Include Files                                */
/*----------------------------------------------------------------------------*/
extern "C"
{

}

#include <cstdint>
#include <map>
#include <string>
#include <vector>
#include "simulation_common.hpp"
#include "move_forward.hpp"

/*----------------------------------------------------------------------------*/
/*                            Private Declarations                            */
/*----------------------------------------------------------------------------*/
/* none */

/*----------------------------------------------------------------------------*/
/*                               Private Globals                              */
/*----------------------------------------------------------------------------*/
/* none */

/*----------------------------------------------------------------------------*/
/*                             Public Definitions                             */
/*----------------------------------------------------------------------------*/
namespace move_forward
{

bool ConfigSweeper::next(void)
{
    if (!initialized_) {
        sweeper.init_sizes({
            dt.size(),
            motor_speed_scale.size(),
            motor1_variance.size(),
            motor2_variance.size(),
            slip_factor.size(),
            wheel_circumference_scale.size(),
            wheel_base_scale.size(),
            motor_speed.size(),
            kp.size(),
            kd.size(),
            pid_shift.size(),
            maze_size_scale.size(),
            ir_reading_scale.size(),
            mouse_angle.size(),
            horizontal_position_variance.size(),
            vertical_position_variance.size()
        });

        initialized_ = true;
    }
    return sweeper.next();
}

Config ConfigSweeper::value(void) const
{
    const auto& idx{sweeper.get_indices()};
    int i{0};

    Config cfg{};

    cfg.dt = dt.at(idx.at(i++));
    cfg.motor_speed_scale = motor_speed_scale.at(idx.at(i++));
    cfg.motor1_variance = motor1_variance.at(idx.at(i++));
    cfg.motor2_variance = motor2_variance.at(idx.at(i++));
    cfg.slip_factor = slip_factor.at(idx.at(i++));
    cfg.wheel_circumference_scale = wheel_circumference_scale.at(idx.at(i++));
    cfg.wheel_base_scale = wheel_base_scale.at(idx.at(i++));
    cfg.motor_speed = motor_speed.at(idx.at(i++));
    cfg.kp = kp.at(idx.at(i++));
    cfg.kd = kd.at(idx.at(i++));
    cfg.pid_shift = pid_shift.at(idx.at(i++));
    cfg.maze_size_scale = maze_size_scale.at(idx.at(i++));
    cfg.ir_reading_scale = ir_reading_scale.at(idx.at(i++));
    cfg.mouse_angle = mouse_angle.at(idx.at(i++));
    cfg.horizontal_position_variance = horizontal_position_variance.at(idx.at(i++));
    cfg.vertical_position_variance = vertical_position_variance.at(idx.at(i++));

    return cfg;
}

} /* move_forward namespace */

/*----------------------------------------------------------------------------*/
/*                             Private Definitions                            */
/*----------------------------------------------------------------------------*/
/* none */
