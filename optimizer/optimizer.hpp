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

struct MetricStats
{
    double mean {0.0};
    double stddev {0.0};
    double min {0.0};
    double max {0.0};
};

class SweepCursor
{
public:
    explicit SweepCursor(const std::vector<SweepParam>& params);

    bool next();
    std::vector<double> values() const;

private:
    std::vector<SweepParam> params_;
    std::vector<int> indices_;
};

/* -------------------------------------------------------------------------- */
/* rotation definitions */
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
    std::vector<double> params;
    RotationResult result;
};

struct RotationAnalysisSummary
{
    double failure_rate {0.0};
    double collision_rate {0.0};

    MetricStats translation_stats;
    MetricStats angle_error_stats;
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

} /* optimizer namespace */

/*----------------------------------------------------------------------------*/
/*                             Public Declarations                            */
/*----------------------------------------------------------------------------*/
namespace optimizer
{

template <typename Result>
std::vector<std::pair<std::vector<double>, Result>> run_parameter_sweep(
        const std::vector<SweepParam>& params,
        const std::function<Result(const std::vector<double>&)>& sim_fn)
{
    SweepCursor cursor(params);
    std::vector<std::pair<std::vector<double>, Result>> results;

    do {
        auto vals = cursor.values();
        results.push_back({vals, sim_fn(vals)});
    } while (cursor.next());

    return results;
}

RotationConfig build_rotation_config(const std::vector<double>& v);
RotationResult run_rotation_simulation(const maze::Maze& maze, const RotationConfig& cfg, double target_angle);
RotationAnalysisSummary analyze_rotation_results(const std::vector<std::pair<std::vector<double>,
        RotationResult>>& trials);
std::vector<RotationParamImpact> analyze_rotation_parameter_impact(const std::vector<SweepParam>& params,
        const std::vector<std::pair<std::vector<double>, RotationResult>>& trials);

} /* optimizer namespace */

#endif /* OPTIMIZER_HPP_ */
