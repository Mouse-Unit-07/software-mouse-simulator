/*-------------------------------- FILE INFO ---------------------------------*/
/* Filename           : side_wall_detection_optimizer.cpp                     */
/*                                                                            */
/* Implementation of a micromouse simulation side_wall_detection_optimizer    */
/*                                                                            */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*                               Include Files                                */
/*----------------------------------------------------------------------------*/
#include <pagmo/algorithms/nsga2.hpp>
#include <pagmo/population.hpp>
#include <pagmo/problem.hpp>
#include <pagmo/types.hpp>
#include <array>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <map>
#include <vector>
#include "simulation_common.hpp"
#include "optimizer_common.hpp"
#include "side_wall_detection.hpp"
#include "side_wall_detection_optimizer.hpp"

/*----------------------------------------------------------------------------*/
/*                            Private Declarations                            */
/*----------------------------------------------------------------------------*/
namespace
{

using PagmoVec = pagmo::vector_double;

std::unordered_map<std::string, std::array<double, 4>> window_start_cache;

} /* unnamed namespace */

/*----------------------------------------------------------------------------*/
/*                               Private Globals                              */
/*----------------------------------------------------------------------------*/
namespace
{

constexpr double W30{0.3};
constexpr double W60{0.6};
constexpr double W90{0.9};
constexpr double W100{1.0};

struct Objectives {
    double w30{};
    double w60{};
    double w90{};
    double w100{};

    PagmoVec to_vec() const
    {
        return {-w30, -w60, -w90, -w100};
    }

    static Objectives from_vec(const PagmoVec& v)
    {
        return {-v.at(0), -v.at(1), -v.at(2), -v.at(3)};
    }
};

class SideWallDetectionUDP {
public:
    SideWallDetectionUDP() = default;

    SideWallDetectionUDP(int sims) : sims_(sims)
    {
        /* no additional logic */
    }

    PagmoVec fitness(const PagmoVec& x) const
    {
        const auto control{side_wall_detection::decode_control(x)};

        std::vector<double> avg_absent;
        std::vector<double> avg_present;

        int steps{0};

        for (int i{0}; i < sims_; ++i) {
            side_wall_detection::Config cfg{
                control,
                side_wall_detection::generate_random_environment()
            };

            const auto r{side_wall_detection::run_simulation(cfg)};

            if (i == 0) {
                steps = static_cast<int>(r.wall_absent_at_step.size());
                avg_absent.assign(steps, 0.0);
                avg_present.assign(steps, 0.0);
            }

            for (int s{0}; s < steps; ++s) {
                avg_absent.at(s) += r.wall_absent_at_step.at(s) ? 1.0 : 0.0;
                avg_present.at(s) += r.wall_present_at_step.at(s) ? 1.0 : 0.0;
            }
        }

        for (int s{0}; s < steps; ++s) {
            avg_absent.at(s) /= sims_;
            avg_present.at(s) /= sims_;
        }

        /* combine absent/present into correctness */
        std::vector<double> correctness(steps);
        for (int s{0}; s < steps; ++s) {
            correctness.at(s) = std::sqrt(avg_absent.at(s) * avg_present.at(s));
        }

        Objectives obj;

        auto r30{side_wall_detection::find_best_window(correctness, W30)};
        auto r60{side_wall_detection::find_best_window(correctness, W60)};
        auto r90{side_wall_detection::find_best_window(correctness, W90)};
        auto r100{side_wall_detection::find_best_window(correctness, W100)};

        obj.w30 = r30.rate;
        obj.w60 = r60.rate;
        obj.w90 = r90.rate;
        obj.w100 = r100.rate;

        /* cache start fractions */
        window_start_cache[optimizer_common::control_to_key(x)] =
            std::array<double, 4>{
                r30.start_fraction,
                r60.start_fraction,
                r90.start_fraction,
                r100.start_fraction
            };

        return obj.to_vec();
    }

    std::pair<PagmoVec, PagmoVec> get_bounds() const
    {
        return side_wall_detection::get_control_bounds();
    }

    pagmo::vector_double::size_type get_nobj() const
    {
        return 4;
    }

private:
    int sims_{100};
};

} /* unnamed namespace */

/*----------------------------------------------------------------------------*/
/*                             Public Definitions                             */
/*----------------------------------------------------------------------------*/
namespace side_wall_detection_optimizer
{

ParetoResult run_side_wall_detection_filter(std::size_t population, std::size_t generations,
                                            int simulations_per_fitness)
{
    pagmo::problem prob{SideWallDetectionUDP{simulations_per_fitness}};
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
    constexpr int W_THRESH{10};

    constexpr int W_RATE{10};
    constexpr int W_START{10};

    out << "===== SIDE WALL DETECTION PARETO FRONT =====\n\n";

    out << std::left
        << std::setw(W_IDX) << "#"
        << std::setw(W_THRESH) << "threshold"
        << " | "
        << std::setw(W_RATE)  << "w30"
        << std::setw(W_START) << "s30"
        << std::setw(W_RATE)  << "w60"
        << std::setw(W_START) << "s60"
        << std::setw(W_RATE)  << "w90"
        << std::setw(W_START) << "s90"
        << std::setw(W_RATE)  << "w100"
        << std::setw(W_START) << "s100"
        << "\n";

    out << std::string(100, '-') << "\n";

    out << std::fixed << std::setprecision(6);

    for (size_t i{0}; i < result.X.size(); ++i) {

        const auto& x{result.X.at(i)};
        const auto ctrl{side_wall_detection::decode_control(x)};
        const auto obj{Objectives::from_vec(result.F.at(i))};

        std::array<double, 4> starts{0.0, 0.0, 0.0, 0.0};

        auto it(window_start_cache.find(optimizer_common::control_to_key(x)));
        if (it != window_start_cache.end()) {
            starts = it->second;
        }

        out << std::left
            << std::setw(W_IDX)    << i
            << std::setw(W_THRESH) << ctrl.reading_threshold
            << " | "
            << std::setw(W_RATE)  << obj.w30
            << std::setw(W_START) << starts.at(0)
            << std::setw(W_RATE)  << obj.w60
            << std::setw(W_START) << starts.at(1)
            << std::setw(W_RATE)  << obj.w90
            << std::setw(W_START) << starts.at(2)
            << std::setw(W_RATE)  << obj.w100
            << std::setw(W_START) << starts.at(3)
            << "\n";
    }
}

} /* side_wall_detection_optimizer namespace */

/*----------------------------------------------------------------------------*/
/*                             Private Definitions                            */
/*----------------------------------------------------------------------------*/
/* none */
