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
#include "move_forward.hpp"
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
        double time{0.0};
        double collision{0.0};
        double timeout{0.0};

        for (int i{0}; i < sims_; ++i) {
            rotation::Config cfg{control, rotation::generate_random_environment()};

            const auto r{rotation::run_simulation(cfg, M_PI / 2)};

            angle += r.final_angle_error;
            translation += r.total_translation;
            time += r.total_time;
            collision += r.collision ? 1.0 : 0.0;
            timeout += r.timeout ? 1.0 : 0.0;
        }

        return {
            angle / sims_,
            translation / sims_,
            time / sims_,
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
        return 5;
    }

    int sims_{100};
};

ParetoResult run_rotation_pareto(std::size_t population, std::size_t generations,
                                 int simulations_per_fitness)
{
    RotationUDP udp{simulations_per_fitness};

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

void write_rotation_pareto_to_file(const std::string& filename, const ParetoResult& result)
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

    out << "===== ROTATION PARETO FRONT =====\n\n";
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
        const auto& x{result.X.at(i)};
        const auto& f{result.F.at(i)};
        const auto ctrl{rotation::decode_control(x)};

        out << std::left
            << std::setw(W_IDX) << i
            << std::setw(W_SPD) << static_cast<int>(ctrl.motor_speed)
            << std::setw(W_KP)  << ctrl.kp
            << std::setw(W_KD)  << ctrl.kd
            << std::setw(W_SH)  << ctrl.pid_shift
            << " | "
            << std::setw(W_ANGLE) << f.at(0)
            << std::setw(W_TRANS) << f.at(1)
            << std::setw(W_TIME)  << f.at(2)
            << std::setw(W_COLL)  << f.at(3)
            << std::setw(W_TO)    << f.at(4)
            << "\n";
    }
}

class MoveForwardUDP {
public:
    MoveForwardUDP() = default;

    MoveForwardUDP(int simulations_per_fitness) : sims_(simulations_per_fitness)
    {
        /* no additional logic */
    }

    PagmoVec fitness(const PagmoVec& x) const
    {
        const auto control{move_forward::decode_control(x)};

        double time{0.0};
        double angle{0.0};
        double horizontal{0.0};
        double vertical{0.0};
        double collision{0.0};
        double timeout{0.0};

        for (int i{0}; i < sims_; ++i) {
            move_forward::Config cfg{control, move_forward::generate_random_environment()};

            const auto r{move_forward::run_simulation(cfg)};

            const auto accumulate = [&](const move_forward::SingleCaseResult& s) {
                time += s.total_time;
                angle += s.total_angle_error;
                horizontal += s.total_horizontal_translation;
                vertical += s.final_vertical_translation;
                collision += s.collision ? 1.0 : 0.0;
                timeout += s.timeout ? 1.0 : 0.0;
            };

            accumulate(r.no_wall);
            accumulate(r.one_wall);
            accumulate(r.two_wall);
        }

        const double denom{static_cast<double>(sims_ * 3)};

        return {
            time / denom,
            angle / denom,
            horizontal / denom,
            vertical / denom,
            collision / denom,
            timeout / denom
        };
    }

    std::pair<PagmoVec, PagmoVec> get_bounds() const
    {
        return move_forward::get_control_bounds();
    }

    pagmo::vector_double::size_type get_nobj() const
    {
        return 6;
    }

    int sims_{100};
};

ParetoResult run_move_forward_pareto(std::size_t population, std::size_t generations,
                                     int simulations_per_fitness)
{
    MoveForwardUDP udp{simulations_per_fitness};

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

void write_move_forward_pareto_to_file(const std::string& filename, const ParetoResult& result)
{
    std::ofstream out(filename);
    if (!out.is_open()) {
        throw std::runtime_error("Failed to open file: " + filename);
    }

    /* column widths */
    constexpr int W_IDX{4};
    constexpr int W_TGT{6};
    constexpr int W_SPD{6};
    constexpr int W_KP{6};
    constexpr int W_KD{6};
    constexpr int W_SH{4};
    constexpr int W_KP_IR{8};
    constexpr int W_KD_IR{8};

    constexpr int W_TIME{12};
    constexpr int W_ANGLE{12};
    constexpr int W_HORIZ{12};
    constexpr int W_VERT{12};
    constexpr int W_COLL{12};
    constexpr int W_TO{12};

    out << std::fixed << std::setprecision(6);

    /* banner */
    out << "===== MOVE FORWARD PARETO FRONT =====\n\n";

    /* header */
    out << std::left
        << std::setw(W_IDX)   << "#"
        << std::setw(W_TGT)   << "tgt"
        << std::setw(W_SPD)   << "spd"
        << std::setw(W_KP)    << "kp"
        << std::setw(W_KD)    << "kd"
        << std::setw(W_SH)    << "sh"
        << std::setw(W_KP_IR) << "kp_ir"
        << std::setw(W_KD_IR) << "kd_ir"
        << " | "
        << std::setw(W_TIME)  << "time"
        << std::setw(W_ANGLE) << "angle"
        << std::setw(W_HORIZ) << "horiz"
        << std::setw(W_VERT)  << "vert"
        << std::setw(W_COLL)  << "collision"
        << std::setw(W_TO)    << "timeout"
        << "\n";

    out << std::string(110, '-') << "\n";

    /* rows */
    for (size_t i{0}; i < result.X.size(); ++i) {
        const auto& x{result.X.at(i)};
        const auto& f{result.F.at(i)};
        const auto ctrl{move_forward::decode_control(x)};

        out << std::left
            << std::setw(W_IDX)   << i
            << std::setw(W_TGT)   << ctrl.single_wall_target
            << std::setw(W_SPD)   << static_cast<int>(ctrl.motor_speed)
            << std::setw(W_KP)    << ctrl.kp
            << std::setw(W_KD)    << ctrl.kd
            << std::setw(W_SH)    << ctrl.pid_shift
            << std::setw(W_KP_IR) << ctrl.kp_ir
            << std::setw(W_KD_IR) << ctrl.kd_ir
            << " | "
            << std::setw(W_TIME)  << f.at(0)
            << std::setw(W_ANGLE) << f.at(1)
            << std::setw(W_HORIZ) << f.at(2)
            << std::setw(W_VERT)  << f.at(3)
            << std::setw(W_COLL)  << f.at(4)
            << std::setw(W_TO)    << f.at(5)
            << "\n";
    }
}

} /* optimizer namespace */

/*----------------------------------------------------------------------------*/
/*                             Private Definitions                            */
/*----------------------------------------------------------------------------*/
/* none */
