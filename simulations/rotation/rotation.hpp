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

struct Config
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

struct Result
{
    double total_time {0.0};
    double final_angle_error {0.0};
    double total_translation {0.0};
    bool collision {false};
    bool timeout {false};
};

struct Trial
{
    Config config;
    Result result;
};

struct ResultsMetrics
{
    optimizer::MetricStats time_stats;
    optimizer::MetricStats angle_error_stats;
    optimizer::MetricStats translation_stats;
    double collision_rate {0.0};
    double failure_rate {0.0};
};

struct CandidateKey {
    int32_t kp;
    int32_t kd;
    int32_t shift;

    bool operator<(const CandidateKey& other) const {
        return std::tie(kp, kd, shift) < std::tie(other.kp, other.kd, other.shift);
    }
};

struct Candidate
{
    CandidateKey key;
    ResultsMetrics results_metrics;
};

} /* rotation namespace */

/*----------------------------------------------------------------------------*/
/*                             Public Declarations                            */
/*----------------------------------------------------------------------------*/
namespace rotation
{

Config build_config(const std::vector<double>& v);
Result run_simulation(const maze::Maze& maze, const Config& cfg, double target_angle);

ResultsMetrics compute_results_metrics(const std::vector<Result>& results);
std::vector<Candidate> build_candidates(const std::vector<Trial>& trials);
void sort_candidates(std::vector<Candidate>& v);

void print_rotation_simulation_results(const std::vector<Candidate>& condidates, ResultsMetrics overall_metrics);
void run_full_rotation_experiment(double target_angle, std::vector<optimizer::SweepConfig> configs);

} /* rotation namespace */

#endif /* ROTATION_HPP_ */
