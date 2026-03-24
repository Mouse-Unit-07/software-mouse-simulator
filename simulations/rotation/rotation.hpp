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

struct RotationConfig
{
    double motor_speed;
    double motor_speed_scale;
    double dt;

    double motor1_variance;
    double motor2_variance;
    double slip_factor;
    double wheel_circumference_scale;
    double wheel_base_scale;

    int32_t kp;
    int32_t kd;
    int32_t pid_shift;
};

struct RotationResult
{
    double total_time {0.0};
    double final_angle_error {0.0};
    double total_translation {0.0};
    bool collision {false};
    bool simulation_failed {false};
};

struct RotationTrial
{
    std::vector<double> configs;
    RotationResult result;
};

struct RotationResultsMetrics
{
    optimizer::MetricStats time_stats;
    optimizer::MetricStats angle_error_stats;
    optimizer::MetricStats translation_stats;
    double collision_rate {0.0};
    double failure_rate {0.0};
};

struct RotationParamImpact
{
    std::string name;

    double correlation_translation {0.0};
    double correlation_angle_error {0.0};

    double failure_rate_low {0.0};
    double failure_rate_high {0.0};

    double collision_rate_low {0.0};
    double collision_rate_high {0.0};
};

struct PdKey {
    int32_t kp;
    int32_t kd;
    int32_t shift;

    bool operator<(const PdKey& other) const {
        return std::tie(kp, kd, shift) < std::tie(other.kp, other.kd, other.shift);
    }
};

struct RotationCandidate
{
    PdKey key;

    optimizer::MetricStats time_stats;
    optimizer::MetricStats angle_error_stats;
    optimizer::MetricStats translation_stats;

    double failure_rate {0.0};
    double collision_rate {0.0};
};

} /* rotation namespace */

/*----------------------------------------------------------------------------*/
/*                             Public Declarations                            */
/*----------------------------------------------------------------------------*/
namespace rotation
{

RotationConfig build_rotation_config(const std::vector<double>& v);
RotationResult run_rotation_simulation(const maze::Maze& maze, const RotationConfig& cfg, double target_angle);
RotationResultsMetrics analyze_rotation_results(const std::vector<RotationTrial>& trials);
std::vector<RotationParamImpact> analyze_rotation_parameter_impact(const std::vector<optimizer::SweepParam>& params,
        const std::vector<RotationTrial>& trials);
std::vector<RotationCandidate> analyze_pd_candidates(const std::vector<RotationTrial>& trials);
void sort_rotation_candidates(std::vector<RotationCandidate>& v);
std::vector<RotationCandidate>get_ranked_pd_candidates(const std::vector<RotationTrial>& trials);

std::vector<optimizer::SweepParam> default_pd_sweep_params(void);

std::vector<RotationTrial> run_pd_sweep(const maze::Maze& maze, double target_angle);

void print_rotation_simulation_results(const std::vector<RotationTrial>& trials);

void run_full_rotation_experiment(double target_angle);

} /* rotation namespace */

#endif /* ROTATION_HPP_ */
