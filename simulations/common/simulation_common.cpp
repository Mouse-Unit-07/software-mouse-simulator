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
#include <stdexcept>
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

void CommonConfigSweeper::init_sizes(std::vector<size_t> sizes)
{
    sizes_ = std::move(sizes);
    for (int x : sizes_) {
        if (x == 0) {
            throw std::invalid_argument("size of zero found- parameter missing in sweeper");
        }
    }

    indices_.assign(sizes_.size(), 0);
}

bool CommonConfigSweeper:: next(void)
{
    if (first_) {
        first_ = false;
        return true;
    }

    for (int i{static_cast<int>(indices_.size()) - 1}; i >= 0; --i) {
        indices_.at(i)++;

        if (indices_.at(i) < sizes_.at(i)) {
            return true;
        }

        indices_.at(i) = 0;
    }

    return false;
}

const std::vector<size_t>& CommonConfigSweeper::get_indices(void) const
{
    return indices_;
}

MetricStats compute_stats(const std::vector<double>& data)
{
    MetricStats s{};

    if (data.empty()) {
        return s;
    }

    double sum{0.0};
    s.min = data.at(0);
    s.max = data.at(0);

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

    double variance{0.0};
    for (double v : data) {
        double d{v - s.mean};
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
