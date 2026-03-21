/*-------------------------------- FILE INFO ---------------------------------*/
/* Filename           : optimizer.hpp                                         */
/*                                                                            */
/* Interface to functions to run micromouse simulations                       */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#ifndef OPTIMIZER_HPP_
#define OPTIMIZER_HPP_

/*----------------------------------------------------------------------------*/
/*                             Public Definitions                             */
/*----------------------------------------------------------------------------*/
namespace optimizer
{

struct SweepParam
{
    std::string name;
    double min;
    double max;
    int steps;
};

struct SimulationConfig
{
    double motor_speed;
    double motor_speed_scale;
    double dt;

    double motor1_variance;
    double motor2_variance;
    double slip_factor;
    double wheel_circumference_scale;
    double wheel_base_scale;
};

struct SimulationResult
{
    double total_time {0.0};
    double final_angle_error {0.0};
    double total_translation {0.0};
    bool collision {false};
    bool simulation_failed {false};
};

} /* optimizer namespace */

/*----------------------------------------------------------------------------*/
/*                             Public Declarations                            */
/*----------------------------------------------------------------------------*/
namespace optimizer
{

std::vector<SimulationResult> run_parameter_sweep(const maze::Maze& maze,
        const std::vector<SweepParam>& params, double target_angle);

} /* optimizer namespace */

#endif /* OPTIMIZER_HPP_ */
