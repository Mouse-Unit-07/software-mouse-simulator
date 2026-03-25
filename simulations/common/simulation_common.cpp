/*-------------------------------- FILE INFO ---------------------------------*/
/* Filename           : simulation_common.cpp                                 */
/*                                                                            */
/* Implementation for common micromouse simulation utilities                  */
/*                                                                            */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*                               Include Files                                */
/*----------------------------------------------------------------------------*/
#include <cmath>
#include <vector>
#include <string>
#include <functional>
#include <algorithm>
#include "simulation_common.hpp"

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
namespace simulation_common
{

SweepCursor::SweepCursor(const std::vector<SweepConfig>& configs)
    : configs_(configs), progress_counter_(configs.size(), 0)
{
    /* no additional logic */
}

bool SweepCursor::next()
{
    for (int i {static_cast<int>(progress_counter_.size()) - 1}; i >= 0; --i) {
        int current_index {progress_counter_[i]};
        int max_steps {configs_[i].steps};

        current_index++;

        if (current_index < max_steps) {
            progress_counter_[i] = current_index;
            return true;
        }

        progress_counter_[i] = 0;
    }

    return false;
}

std::vector<double> SweepCursor::values() const
{
    std::vector<double> vals;
    vals.reserve(configs_.size());

    for (size_t i = 0; i < configs_.size(); ++i) {
        const SweepConfig& c = configs_[i];
        int index = progress_counter_[i];

        double value;

        if (c.steps <= 1) {
            value = c.min;
        } else {
            double fraction = static_cast<double>(index) / (c.steps - 1);
            value = c.min + fraction * (c.max - c.min);
        }

        vals.push_back(value);
    }

    return vals;
}

MetricStats compute_stats(const std::vector<double>& data)
{
    MetricStats s{};

    if (data.empty()) {
        return s;
    }

    double sum {0.0};
    s.min = data[0];
    s.max = data[0];

    for (double v : data) {
        sum += v;
        if (v < s.min) {
            s.min = v;
        }
        if (v > s.max) {
            s.max = v;
        }
    }

    s.mean = sum / data.size();

    double variance {0.0};
    for (double v : data) {
        double d {v - s.mean};
        variance += d * d;
    }

    s.stddev = sqrt(variance / data.size());
    return s;
}

} /* simulation_common namespace */

/*----------------------------------------------------------------------------*/
/*                             Private Definitions                            */
/*----------------------------------------------------------------------------*/
/* none */
