/*-------------------------------- FILE INFO ---------------------------------*/
/* Filename           : front_wall_detection_optimizer.cpp                    */
/*                                                                            */
/* Implementation of a micromouse simulation front_wall_detection_optimizer   */
/*                                                                            */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*                               Include Files                                */
/*----------------------------------------------------------------------------*/
#include <pagmo/algorithms/nsga2.hpp>
#include <pagmo/population.hpp>
#include <pagmo/problem.hpp>
#include <pagmo/types.hpp>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <unordered_map>
#include <vector>
#include "simulation_common.hpp"
#include "optimizer_common.hpp"
#include "front_wall_detection.hpp"
#include "front_wall_detection_optimizer.hpp"

/*----------------------------------------------------------------------------*/
/*                            Private Declarations                            */
/*----------------------------------------------------------------------------*/
namespace
{

using PagmoVec = pagmo::vector_double;

std::unordered_map<std::string, std::pair<double, double>> rate_cache{};

struct VerticalVarianceWindow {
    double max_positive{-std::numeric_limits<double>::infinity()};
    double min_negative{std::numeric_limits<double>::infinity()};
};

std::unordered_map<std::string, VerticalVarianceWindow> vertical_window_cache{};

} /* unnamed namespace */

/*----------------------------------------------------------------------------*/
/*                               Private Globals                              */
/*----------------------------------------------------------------------------*/
namespace
{

struct Objectives {
    double combined_rate{};
    double vertical_window{};

    PagmoVec to_vec() const
    {
        return {-combined_rate, -vertical_window};
    }

    static Objectives from_vec(const PagmoVec& v)
    {
        Objectives obj{};
        obj.combined_rate = -v.at(0);
        obj.vertical_window = -v.at(1);
        return obj;
    }
};

class FrontWallDetectionUDP {
public:
    FrontWallDetectionUDP() = default;

    FrontWallDetectionUDP(int sims) : sims_(sims)
    {
        /* no additional logic */
    }

    PagmoVec fitness(const PagmoVec& x) const
    {
        const auto control{front_wall_detection::decode_control(x)};

        double absent{0.0};
        double present{0.0};
        double max_positive{0.0};
        double min_negative{0.0};

        for (int i{0}; i < sims_; ++i) {
            front_wall_detection::Config cfg{
                control,
                front_wall_detection::generate_random_environment()
            };

            const auto r{front_wall_detection::run_simulation(cfg)};

            absent += r.identified_absent_wall ? 1.0 : 0.0;
            present += r.identified_present_wall ? 1.0 : 0.0;

            /* cache valid vertical positioning window */
            if (r.identified_absent_wall && r.identified_present_wall) {
                const double variance{cfg.env_cfg.vertical_position_variance};

                if (variance >= 0.0) {
                    if (variance > max_positive) {
                        max_positive = variance;
                    }
                } else {
                    if (variance < min_negative) {
                        min_negative = variance;
                    }
                }
            }
        }

        const double absent_rate{absent / sims_};
        const double present_rate{present / sims_};
        const double combined_rate{std::sqrt(absent_rate * present_rate)};

        const double vertical_window{max_positive - min_negative};

        /* cache metrics for reporting */
        const std::string key{optimizer_common::control_to_key(x)};

        rate_cache[key] = {absent_rate, present_rate};
        vertical_window_cache[key] = {max_positive, min_negative};

        Objectives obj{};
        obj.combined_rate = combined_rate;
        obj.vertical_window = vertical_window;

        return obj.to_vec();
    }

    std::pair<PagmoVec, PagmoVec> get_bounds() const
    {
        return front_wall_detection::get_control_bounds();
    }

    pagmo::vector_double::size_type get_nobj() const
    {
        return 2;
    }

private:
    int sims_{100};
};

} /* unnamed namespace */

/*----------------------------------------------------------------------------*/
/*                             Public Definitions                             */
/*----------------------------------------------------------------------------*/
namespace front_wall_detection_optimizer
{

ParetoResult run_front_wall_detection_staged(std::size_t population, std::size_t generations,
                                             int simulations_per_fitness)
{
    pagmo::problem prob{FrontWallDetectionUDP{simulations_per_fitness}};
    pagmo::algorithm algo{pagmo::nsga2{}};
    pagmo::population pop{prob, population};

    for (std::size_t i{0}; i < generations; ++i) {
        pop = algo.evolve(pop);
    }

    return {pop.get_x(), pop.get_f()};
}

void write_pareto_to_file(const std::string& filename, const ParetoResult& result)
{
    auto out{optimizer_common::open_output_file(filename)};

    constexpr int W_IDX{4};
    constexpr int W_THRESH{12};
    constexpr int W_COMB{16};
    constexpr int W_WINDOW{16};
    constexpr int W_ABS{16};
    constexpr int W_PRE{16};
    constexpr int W_NEG{16};
    constexpr int W_POS{16};

    out << "===== FRONT WALL DETECTION PARETO FRONT =====\n\n";

    out << std::left
        << std::setw(W_IDX)    << "#"
        << std::setw(W_THRESH) << "threshold"
        << " | "
        << std::setw(W_COMB)   << "combined"
        << std::setw(W_WINDOW) << "window"
        << std::setw(W_ABS)    << "absent_rate"
        << std::setw(W_PRE)    << "present_rate"
        << std::setw(W_NEG)    << "neg_vert"
        << std::setw(W_POS)    << "pos_vert"
        << "\n";

    out << std::string(140, '-') << "\n";

    for (size_t i{0}; i < result.X.size(); ++i) {
        const auto& x{result.X.at(i)};
        const auto& f{result.F.at(i)};
        const auto ctrl{front_wall_detection::decode_control(x)};
        const auto obj{Objectives::from_vec(f)};
        double absent_rate{0.0};
        double present_rate{0.0};
        double max_positive{0.0};
        double min_negative{0.0};

        const std::string key{optimizer_common::control_to_key(x)};

        auto rate_it{rate_cache.find(key)};
        if (rate_it != rate_cache.end()) {
            absent_rate = rate_it->second.first;
            present_rate = rate_it->second.second;
        }

        auto it{vertical_window_cache.find(key)};
        if (it != vertical_window_cache.end()) {
            if (it->second.max_positive != -std::numeric_limits<double>::infinity()) {
                max_positive = it->second.max_positive;
            }
            if (it->second.min_negative != std::numeric_limits<double>::infinity()) {
                min_negative = it->second.min_negative;
            }
        }

        out << std::left
            << std::setw(W_IDX)    << i
            << std::setw(W_THRESH) << ctrl.reading_threshold
            << " | "
            << std::setw(W_COMB)   << obj.combined_rate
            << std::setw(W_WINDOW) << obj.vertical_window
            << std::setw(W_ABS)    << absent_rate
            << std::setw(W_PRE)    << present_rate
            << std::setw(W_NEG)    << min_negative
            << std::setw(W_POS)    << max_positive
            << "\n";
    }
}

} /* front_wall_detection_optimizer namespace */

/*----------------------------------------------------------------------------*/
/*                             Private Definitions                            */
/*----------------------------------------------------------------------------*/
/* none */
