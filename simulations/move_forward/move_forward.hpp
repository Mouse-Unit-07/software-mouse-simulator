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

    uint32_t single_wall_target;
    uint8_t motor_speed;
    int32_t kp;
    int32_t kd;
    int32_t pid_shift;
    int32_t kp_ir;
    int32_t kd_ir;
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

    std::vector<uint32_t> single_wall_target;
    std::vector<uint8_t> motor_speed;
    std::vector<int32_t> kp;
    std::vector<int32_t> kd;
    std::vector<int32_t> pid_shift;
    std::vector<int32_t> kp_ir;
    std::vector<int32_t> kd_ir;

    bool next(void);
    Config value(void) const;

private:
    simulation_common::CommonConfigSweeper sweeper;
    bool initialized_{false};
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

struct Trial {
    Config config;
    Result result;
};

struct SingleCaseResultsMetrics {
    simulation_common::MetricStats time_stats;
    simulation_common::MetricStats angle_error_stats;
    simulation_common::MetricStats horizontal_translation_stats;
    simulation_common::MetricStats vertical_translation_stats;
    double collision_rate{0.0};
    double timeout_rate{0.0};
};

struct ResultsMetrics {
    SingleCaseResultsMetrics no_wall_metrics;
    SingleCaseResultsMetrics one_wall_metrics;
    SingleCaseResultsMetrics two_wall_metrics;
};

} /* move_forward namespace */

/*----------------------------------------------------------------------------*/
/*                             Public Declarations                            */
/*----------------------------------------------------------------------------*/
namespace move_forward
{

SingleCaseResult run_single_simulation(const Config& cfg, const maze::Maze& maze,
                                       enum wall_mode mode);
Result run_simulation(const Config& cfg);

ResultsMetrics compute_results_metrics(const std::vector<Result>& results);

} /* move_forward namespace */

#endif /* MOVE_FORWARD_HPP_ */
