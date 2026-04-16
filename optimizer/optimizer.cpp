/*-------------------------------- FILE INFO ---------------------------------*/
/* Filename           : optimizer.cpp                                         */
/*                                                                            */
/* Implementation of a micromouse simulation optimizer                        */
/*                                                                            */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*                               Include Files                                */
/*----------------------------------------------------------------------------*/
#include <pagmo/algorithms/nsga2.hpp>
#include <pagmo/population.hpp>
#include <pagmo/problem.hpp>
#include <pagmo/types.hpp>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <map>
#include <vector>
#include "simulation_common.hpp"
#include "rotation.hpp"
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

using PagmoVec = pagmo::vector_double;

class RotationUDP {
public:
    PagmoVec fitness(const PagmoVec& x) const
    {
        const auto control{rotation::decode_control(x)};

        constexpr int N{1000};
        double angle{0.0};
        double translation{0.0};
        double time{0.0};
        double collision{0.0};
        double timeout{0.0};

        for (int i{0}; i < N; ++i) {
            rotation::Config cfg{rotation::merge_control_and_environment(
                control, rotation::generate_random_environment())};

            const auto r{rotation::run_simulation(cfg, M_PI / 2)};

            angle += r.final_angle_error;
            translation += r.total_translation;
            time += r.total_time;
            collision += r.collision ? 1.0 : 0.0;
            timeout += r.timeout ? 1.0 : 0.0;
        }

        return {
            angle / N,
            translation / N,
            time / N,
            collision / N,
            timeout / N
        };
    }

    std::pair<PagmoVec, PagmoVec> get_bounds() const
    {
        return rotation::get_control_bounds();
    }

    pagmo::vector_double::size_type get_nobj() const
    {
        return 5;
    }
};

ParetoResult run_rotation_pareto(std::size_t population, std::size_t generations)
{
    RotationUDP udp;

    pagmo::problem prob{udp};

    pagmo::algorithm algo{pagmo::nsga2{}};

    pagmo::population pop{prob, population};

    for (std::size_t i{0}; i < generations; ++i) {
        pop = algo.evolve(pop);
    }

    ParetoResult out;
    out.X = pop.get_x();
    out.F = pop.get_f();

    return out;
}

void write_pareto_to_file(const std::string& filename, const ParetoResult& result)
{
    std::ofstream out(filename);
    if (!out.is_open()) {
        throw std::runtime_error("Failed to open file: " + filename);
    }

    constexpr int W_IDX{4};
    constexpr int W_SPD{6};
    constexpr int W_KP{6};
    constexpr int W_KD{6};
    constexpr int W_SH{4};

    constexpr int W_ANGLE{12};
    constexpr int W_TRANS{12};
    constexpr int W_TIME{12};
    constexpr int W_COLL{12};
    constexpr int W_TO{12};

    out << std::fixed << std::setprecision(6);

    out << "===== PARETO FRONT =====\n\n";
    out << std::left
        << std::setw(W_IDX)  << "#"
        << std::setw(W_SPD)  << "spd"
        << std::setw(W_KP)   << "kp"
        << std::setw(W_KD)   << "kd"
        << std::setw(W_SH)   << "sh"
        << " | "
        << std::setw(W_ANGLE) << "angle"
        << std::setw(W_TRANS) << "translation"
        << std::setw(W_TIME)  << "time"
        << std::setw(W_COLL)  << "collision"
        << std::setw(W_TO)    << "timeout"
        << "\n";

    out << std::string(80, '-') << "\n";

    for (size_t i{0}; i < result.X.size(); ++i) {
        const auto& x{result.X[i]};
        const auto& f{result.F[i]};
        const auto ctrl{rotation::decode_control(x)};

        out << std::left
            << std::setw(W_IDX) << i
            << std::setw(W_SPD) << static_cast<int>(ctrl.motor_speed)
            << std::setw(W_KP)  << ctrl.kp
            << std::setw(W_KD)  << ctrl.kd
            << std::setw(W_SH)  << ctrl.pid_shift
            << " | "
            << std::setw(W_ANGLE) << f[0]
            << std::setw(W_TRANS) << f[1]
            << std::setw(W_TIME)  << f[2]
            << std::setw(W_COLL)  << f[3]
            << std::setw(W_TO)    << f[4]
            << "\n";
    }
}

} /* optimizer namespace */

/*----------------------------------------------------------------------------*/
/*                             Private Definitions                            */
/*----------------------------------------------------------------------------*/
/* none */
