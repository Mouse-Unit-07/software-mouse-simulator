/*-------------------------------- FILE INFO ---------------------------------*/
/* Filename           : optimizer.cpp                                         */
/*                                                                            */
/* Implementation for micromouse simulation variable sweeper and analysis     */
/* helpers                                                                    */
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
#include "optimizer.hpp"

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
namespace optimizer
{

SweepCursor::SweepCursor(const std::vector<SweepParam>& params)
    : params_(params), indices_(params.size(), 0)
{
    /* no additional logic */
}

bool SweepCursor::next()
{
    for (int i {(int)indices_.size() - 1}; i >= 0; i--) {
        indices_[i]++;

        if (indices_[i] < params_[i].steps)
            return true;

        indices_[i] = 0;
    }

    return false;
}

std::vector<double> SweepCursor::values() const
{
    std::vector<double> vals;
    vals.reserve(params_.size());

    for (size_t i {0}; i < params_.size(); i++) {
        const auto& p {params_[i]};

        double t {(p.steps == 1) ? 0.0 : static_cast<double>(indices_[i]) / (p.steps - 1)};

        vals.push_back(p.min + t * (p.max - p.min));
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

double compute_correlation(const std::vector<double>& x, const std::vector<double>& y)
{
    if ((x.size() != y.size()) || x.empty()) {
        return 0.0;
    }

    double mean_x {0.0};
    double mean_y {0.0};

    for (size_t i = 0; i < x.size(); i++) {
        mean_x += x[i];
        mean_y += y[i];
    }

    mean_x /= x.size();
    mean_y /= y.size();

    double num {0.0};
    double den_x {0.0};
    double den_y {0.0};

    for (size_t i = 0; i < x.size(); i++) {
        double dx {x[i] - mean_x};
        double dy {y[i] - mean_y};

        num += dx * dy;
        den_x += dx * dx;
        den_y += dy * dy;
    }

    return num / sqrt(den_x * den_y + 1e-9);
}

} /* optimizer namespace */

/*----------------------------------------------------------------------------*/
/*                             Private Definitions                            */
/*----------------------------------------------------------------------------*/
/* none */
