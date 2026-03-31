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

MetricStats compute_stats(const std::vector<double>& data)
{
    MetricStats s{};

    if (data.empty()) {
        return s;
    }

    double sum{0.0};
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
