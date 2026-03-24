/*-------------------------------- FILE INFO ---------------------------------*/
/* Filename           : optimizer.hpp                                         */
/*                                                                            */
/* Interface to functions to run micromouse simulation variable sweeper and   */
/* analysis helpers                                                           */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#ifndef OPTIMIZER_HPP_
#define OPTIMIZER_HPP_

/*----------------------------------------------------------------------------*/
/*                             Public Definitions                             */
/*----------------------------------------------------------------------------*/
namespace optimizer
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

} /* optimizer namespace */

/*----------------------------------------------------------------------------*/
/*                             Public Declarations                            */
/*----------------------------------------------------------------------------*/
namespace optimizer
{

MetricStats compute_stats(const std::vector<double>& data);

} /* optimizer namespace */

#endif /* OPTIMIZER_HPP_ */
