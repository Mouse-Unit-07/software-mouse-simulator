/*-------------------------------- FILE INFO ---------------------------------*/
/* Filename           : rotation_optimizer.cpp                                */
/*                                                                            */
/* Implementation of a micromouse simulation rotation_optimizer               */
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
#include "rotation.hpp"
#include "rotation_optimizer.hpp"

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
/* none */

/*----------------------------------------------------------------------------*/
/*                             Public Definitions                             */
/*----------------------------------------------------------------------------*/
namespace rotation_optimizer
{

class RotationUDP {
public:
    RotationUDP() = default;

    RotationUDP(int simulations_per_fitness) : sims_(simulations_per_fitness)
    {
        /* no additional logic */
    }

    PagmoVec fitness(const PagmoVec& x) const
    {
        const auto control{rotation::decode_control(x)};

        double angle{0.0};
        double translation{0.0};
        double collision{0.0};
        double timeout{0.0};

        for (int i{0}; i < sims_; ++i) {
            rotation::Config cfg{control, rotation::generate_random_environment()};

            const auto r{rotation::run_simulation(cfg)};

            angle += r.final_angle_error;
            translation += r.total_translation;
            collision += r.collision ? 1.0 : 0.0;
            timeout += r.timeout ? 1.0 : 0.0;
        }

        return {
            angle / sims_,
            translation / sims_,
            collision / sims_,
            timeout / sims_
        };
    }

    std::pair<PagmoVec, PagmoVec> get_bounds() const
    {
        return rotation::get_control_bounds();
    }

    pagmo::vector_double::size_type get_nobj() const
    {
        return 4;
    }

    int sims_{100};
};

ParetoResult run_rotation_pareto(std::size_t population, std::size_t generations,
                                 int simulations_per_fitness)
{
    pagmo::problem prob{RotationUDP{simulations_per_fitness}};
    pagmo::algorithm algo{pagmo::nsga2{}};
    pagmo::population pop{prob, population};

    for (std::size_t i{0}; i < generations; ++i) {
        pop = algo.evolve(pop);
    }

    return {pop.get_x(), pop.get_f()};
}

void write_rotation_pareto_to_file(const std::string& filename, const ParetoResult& result)
{
    auto out{optimizer_common::open_output_file(filename)};

    constexpr int W_IDX{4};
    constexpr int W_SPD{6};
    constexpr int W_KP_V{6};
    constexpr int W_KD_V{6};
    constexpr int W_KP_A{6};
    constexpr int W_KD_A{6};
    constexpr int W_SC{4};

    constexpr int W_ANGLE{12};
    constexpr int W_TRANS{12};
    constexpr int W_COLL{12};
    constexpr int W_TO{12};

    out << "===== ROTATION PARETO FRONT =====\n\n";
    out << std::left
        << std::setw(W_IDX) << "#"
        << std::setw(W_SPD) << "spd"
        << std::setw(W_KP_V) << "kp_v"
        << std::setw(W_KD_V) << "kd_v"
        << std::setw(W_KP_A) << "kp_a"
        << std::setw(W_KD_A) << "kd_a"
        << std::setw(W_SC) << "sc"
        << " | "
        << std::setw(W_ANGLE) << "angle"
        << std::setw(W_TRANS) << "translation"
        << std::setw(W_COLL) << "collision"
        << std::setw(W_TO) << "timeout"
        << "\n";

    out << std::string(100, '-') << "\n";

    for (size_t i{0}; i < result.X.size(); ++i) {
        const auto& x{result.X.at(i)};
        const auto& f{result.F.at(i)};
        const auto ctrl{rotation::decode_control(x)};

        out << std::left
            << std::setw(W_IDX) << i
            << std::setw(W_SPD) << static_cast<int>(ctrl.motor_speed)
            << std::setw(W_KP_V) << ctrl.kp_velocity
            << std::setw(W_KD_V) << ctrl.kd_velocity
            << std::setw(W_KP_A) << ctrl.kp_angle
            << std::setw(W_KD_A) << ctrl.kd_angle
            << std::setw(W_SC) << ctrl.pid_scale
            << " | "
            << std::setw(W_ANGLE) << f.at(0)
            << std::setw(W_TRANS) << f.at(1)
            << std::setw(W_COLL) << f.at(2)
            << std::setw(W_TO) << f.at(3)
            << "\n";
    }
}

} /* rotation_optimizer namespace */

/*----------------------------------------------------------------------------*/
/*                             Private Definitions                            */
/*----------------------------------------------------------------------------*/
/* none */
