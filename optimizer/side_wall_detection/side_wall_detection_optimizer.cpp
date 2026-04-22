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

std::unordered_map<std::string, std::pair<double, double>> rate_cache;

} /* unnamed namespace */

/*----------------------------------------------------------------------------*/
/*                               Private Globals                              */
/*----------------------------------------------------------------------------*/
namespace
{

struct Objectives {
    double combined_rate{};
    double start_offset{};

    PagmoVec to_vec() const
    {
        return {-combined_rate, start_offset};
    }

    static Objectives from_vec(const PagmoVec& v)
    {
        Objectives obj{};
        obj.combined_rate = -v.at(0);
        obj.start_offset = v.at(1);
        return obj;
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

        int absent_correct_count{0};
        int present_correct_count{0};

        for (int i{0}; i < sims_; ++i) {
            side_wall_detection::Config cfg{
                control,
                side_wall_detection::generate_random_environment()
            };

            const auto r{side_wall_detection::run_simulation(cfg)};

            absent_correct_count += r.wall_absent_correct ? 1 : 0;
            present_correct_count += r.wall_present_correct ? 1 : 0;
        }

        const double absent_rate{static_cast<double>(absent_correct_count) / sims_};
        const double present_rate{static_cast<double>(present_correct_count) / sims_};
        const double combined_rate{std::sqrt(absent_rate * present_rate)};

        rate_cache[optimizer_common::control_to_key(x)] = {absent_rate, present_rate};

        Objectives obj;
        obj.combined_rate = combined_rate;
        obj.start_offset = control.reading_start_offset;

        return obj.to_vec();
    }

    std::pair<PagmoVec, PagmoVec> get_bounds() const
    {
        return side_wall_detection::get_control_bounds();
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
        << std::setw(W_START) << "start_off"
        << " | "
        << std::setw(W_RATE) << "combined"
        << std::setw(W_RATE) << "absent"
        << std::setw(W_RATE) << "present"
        << "\n";

    out << std::string(100, '-') << "\n";

    out << std::fixed << std::setprecision(6);

    for (size_t i{0}; i < result.X.size(); ++i) {

        const auto& x{result.X.at(i)};
        const auto ctrl{side_wall_detection::decode_control(x)};
        const auto obj{Objectives::from_vec(result.F.at(i))};

        double absent{0.0};
        double present{0.0};

        auto it = rate_cache.find(optimizer_common::control_to_key(x));
        if (it != rate_cache.end()) {
            absent = it->second.first;
            present = it->second.second;
        }

        out << std::left
            << std::setw(W_IDX) << i
            << std::setw(W_THRESH) << ctrl.reading_threshold
            << std::setw(W_START) << ctrl.reading_start_offset
            << " | "
            << std::setw(W_RATE) << obj.combined_rate
            << std::setw(W_RATE) << absent
            << std::setw(W_RATE) << present
            << "\n";
    }
}

} /* side_wall_detection_optimizer namespace */

/*----------------------------------------------------------------------------*/
/*                             Private Definitions                            */
/*----------------------------------------------------------------------------*/
/* none */
