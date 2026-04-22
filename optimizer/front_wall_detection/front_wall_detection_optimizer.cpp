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
#include <map>
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

} /* unnamed namespace */

/*----------------------------------------------------------------------------*/
/*                               Private Globals                              */
/*----------------------------------------------------------------------------*/
namespace
{

struct Objectives {
    double absent{};
    double present{};

    PagmoVec to_vec() const
    {
        /* negate for minimization */
        return {-absent, -present};
    }

    static Objectives from_vec(const PagmoVec& v)
    {
        return {-v.at(0), -v.at(1)};
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

        for (int i{0}; i < sims_; ++i) {
            front_wall_detection::Config cfg{
                control,
                front_wall_detection::generate_random_environment()
            };

            const auto r{front_wall_detection::run_simulation(cfg)};

            absent += r.identified_absent_wall ? 1.0 : 0.0;
            present += r.identified_present_wall ? 1.0 : 0.0;
        }

        Objectives obj{};
        obj.absent = absent / sims_;
        obj.present = present / sims_;

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
    constexpr int W_ABS{16};
    constexpr int W_PRE{16};

    out << "===== FRONT WALL DETECTION PARETO FRONT =====\n\n";

    out << std::left
        << std::setw(W_IDX)    << "#"
        << std::setw(W_THRESH) << "threshold"
        << " | "
        << std::setw(W_ABS)    << "absent_rate"
        << std::setw(W_PRE)    << "present_rate"
        << "\n";

    out << std::string(60, '-') << "\n";

    for (size_t i{0}; i < result.X.size(); ++i) {
        const auto& x{result.X.at(i)};
        const auto& f{result.F.at(i)};
        const auto ctrl{front_wall_detection::decode_control(x)};
        const auto obj{Objectives::from_vec(f)};

        out << std::left
            << std::setw(W_IDX)    << i
            << std::setw(W_THRESH) << ctrl.reading_threshold
            << " | "
            << std::setw(W_ABS)    << obj.absent
            << std::setw(W_PRE)    << obj.present
            << "\n";
    }
}

} /* front_wall_detection_optimizer namespace */

/*----------------------------------------------------------------------------*/
/*                             Private Definitions                            */
/*----------------------------------------------------------------------------*/
/* none */
