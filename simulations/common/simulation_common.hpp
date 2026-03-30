/*-------------------------------- FILE INFO ---------------------------------*/
/* Filename           : simulation_common.hpp                                 */
/*                                                                            */
/* Interface to common micromouse simulation utilities                        */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#ifndef SIMULATION_COMMON_HPP_
#define SIMULATION_COMMON_HPP_

/*----------------------------------------------------------------------------*/
/*                             Public Definitions                             */
/*----------------------------------------------------------------------------*/
namespace simulation_common
{

struct SweepConfig
{
    std::string name;
    double min;
    double max;
    int steps;
};

class SweepCursor
{
public:
    explicit SweepCursor(const std::vector<SweepConfig>& configs);

    bool next();
    std::vector<double> values() const;

private:
    std::vector<SweepConfig> configs_;
    std::vector<int> progress_counter_;
};

template <typename Result>
std::vector<std::pair<std::vector<double>, Result>> run_config_sweep(
        const std::vector<SweepConfig>& configs,
        const std::function<Result(const std::vector<double>&)>& sim_fn)
{
    SweepCursor cursor(configs);
    std::vector<std::pair<std::vector<double>, Result>> results;

    do {
        auto vals = cursor.values();
        results.push_back({vals, sim_fn(vals)});
    } while (cursor.next());

    return results;
}

template <typename T>
std::vector<T> generate_sweep_values(T min, T max, int steps)
{
    std::vector<T> values;

    if (steps <= 0) return values;

    if (steps == 1) {
        values.push_back(min);
        return values;
    }

    if (steps == 2) {
        values.push_back(min);
        values.push_back(max);
        return values;
    }

    values.reserve(steps);

    for (int i = 0; i < steps; ++i) {
        double t = static_cast<double>(i) / (steps - 1);
        values.push_back(static_cast<T>(min + t * (max - min)));
    }

    return values;
}

struct MetricStats
{
    double mean {0.0};
    double stddev {0.0};
    double min {0.0};
    double max {0.0};
};

template <typename Trials, typename Fn>
std::vector<double> extract_metric(const Trials& trials, Fn fn)
{
    std::vector<double> out;
    out.reserve(trials.size());

    for (const auto& t : trials) {
        out.push_back(fn(t));
    }

    return out;
}

template <typename Trials, typename Pred>
double compute_rate(const Trials& trials, Pred pred)
{
    if (trials.empty()) {
        return 0.0;
    }

    int count {0};
    for (const auto& t : trials) {
        if (pred(t)) {
            count++;
        }
    }

    return static_cast<double>(count) / trials.size();
}

} /* simulation_common namespace */

/*----------------------------------------------------------------------------*/
/*                             Public Declarations                            */
/*----------------------------------------------------------------------------*/
namespace simulation_common
{

MetricStats compute_stats(const std::vector<double>& data);

} /* simulation_common namespace */

#endif /* SIMULATION_COMMON_HPP_ */
