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

template <typename Trials, typename ValueFn, typename Pred>
std::pair<double, double> compute_split_rate(const Trials& trials, ValueFn value_fn, Pred pred)
{
    if (trials.empty()) {
        return {0.0, 0.0};
    }

    std::vector<double> values;
    values.reserve(trials.size());

    for (const auto& t : trials) {
        values.push_back(value_fn(t));
    }

    std::sort(values.begin(), values.end());
    double mid {values[values.size() / 2]};

    int low_total {0}, high_total {0};
    int low_count {0}, high_count {0};

    for (const auto& t : trials) {
        if (value_fn(t) < mid) {
            low_total++;
            if (pred(t)) {
                low_count++;
            }
        } else {
            high_total++;
            if (pred(t)) {
                high_count++;
            }
        }
    }

    double low_rate {(low_total > 0) ? static_cast<double>(low_count) / low_total : 0.0};
    double high_rate {(high_total > 0) ? static_cast<double>(high_count) / high_total : 0.0};

    return {low_rate, high_rate};
}

} /* optimizer namespace */

/*----------------------------------------------------------------------------*/
/*                             Public Declarations                            */
/*----------------------------------------------------------------------------*/
namespace optimizer
{

MetricStats compute_stats(const std::vector<double>& data);
double compute_correlation(const std::vector<double>& x, const std::vector<double>& y);

} /* optimizer namespace */

#endif /* OPTIMIZER_HPP_ */
