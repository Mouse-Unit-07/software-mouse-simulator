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
namespace
{

std::ofstream open_output_file(const std::string& filename);

} /* unnamed namespace */

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

template <typename UDP>
ParetoResult run_pareto_impl(UDP&& udp, std::size_t population, std::size_t generations)
{
    pagmo::problem prob{std::forward<UDP>(udp)};
    pagmo::algorithm algo{pagmo::nsga2{}};
    pagmo::population pop{prob, population};

    for (std::size_t i{0}; i < generations; ++i) {
        pop = algo.evolve(pop);
    }

    return {pop.get_x(), pop.get_f()};
}

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
    return run_pareto_impl(RotationUDP{simulations_per_fitness}, population, generations);
}

void write_rotation_pareto_to_file(const std::string& filename, const ParetoResult& result)
{
    auto out{open_output_file(filename)};

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

        double angle{0.0};
        double horizontal{0.0};
        double vertical{0.0};
        double collision{0.0};
        double timeout{0.0};

        for (int i{0}; i < sims_; ++i) {
            move_forward::Config cfg{control, move_forward::generate_random_environment()};

            const auto r{move_forward::run_simulation(cfg)};

            const auto accumulate = [&](const move_forward::SingleCaseResult& s) {
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
        return 5;
    }

    int sims_{100};
};

ParetoResult run_move_forward_pareto(std::size_t population, std::size_t generations,
                                     int simulations_per_fitness)
{
    return run_pareto_impl(MoveForwardUDP{simulations_per_fitness}, population, generations);
}

void write_move_forward_pareto_to_file(const std::string& filename, const ParetoResult& result)
{
    auto out{open_output_file(filename)};

    /* column widths */
    constexpr int W_IDX{4};
    constexpr int W_TGT{6};
    constexpr int W_SPD{6};
    constexpr int W_KP_V{6};
    constexpr int W_KD_V{6};
    constexpr int W_KP_A{6};
    constexpr int W_KD_A{6};
    constexpr int W_SC{4};
    constexpr int W_KP_IR{8};
    constexpr int W_KD_IR{8};

    constexpr int W_ANGLE{12};
    constexpr int W_HORIZ{12};
    constexpr int W_VERT{12};
    constexpr int W_COLL{12};
    constexpr int W_TO{12};

    /* banner */
    out << "===== MOVE FORWARD PARETO FRONT =====\n\n";

    /* header */
    out << std::left
        << std::setw(W_IDX)   << "#"
        << std::setw(W_TGT)   << "tgt"
        << std::setw(W_SPD)   << "spd"
        << std::setw(W_KP_V) << "kp_v"
        << std::setw(W_KD_V) << "kd_v"
        << std::setw(W_KP_A) << "kp_a"
        << std::setw(W_KD_A) << "kd_a"
        << std::setw(W_SC)    << "sc"
        << std::setw(W_KP_IR) << "kp_ir"
        << std::setw(W_KD_IR) << "kd_ir"
        << " | "
        << std::setw(W_ANGLE) << "angle"
        << std::setw(W_HORIZ) << "horiz"
        << std::setw(W_VERT)  << "vert"
        << std::setw(W_COLL)  << "collision"
        << std::setw(W_TO)    << "timeout"
        << "\n";

    out << std::string(130, '-') << "\n";

    /* rows */
    for (size_t i{0}; i < result.X.size(); ++i) {
        const auto& x{result.X.at(i)};
        const auto& f{result.F.at(i)};
        const auto ctrl{move_forward::decode_control(x)};

        out << std::left
            << std::setw(W_IDX)   << i
            << std::setw(W_TGT)   << ctrl.single_wall_target
            << std::setw(W_SPD)   << static_cast<int>(ctrl.motor_speed)
            << std::setw(W_KP_V) << ctrl.kp_velocity
            << std::setw(W_KD_V) << ctrl.kd_velocity
            << std::setw(W_KP_A) << ctrl.kp_angle
            << std::setw(W_KD_A) << ctrl.kd_angle
            << std::setw(W_SC)    << ctrl.pid_scale
            << std::setw(W_KP_IR) << ctrl.kp_ir
            << std::setw(W_KD_IR) << ctrl.kd_ir
            << " | "
            << std::setw(W_ANGLE) << f.at(0)
            << std::setw(W_HORIZ) << f.at(1)
            << std::setw(W_VERT)  << f.at(2)
            << std::setw(W_COLL)  << f.at(3)
            << std::setw(W_TO)    << f.at(4)
            << "\n";
    }
}

} /* optimizer namespace */

/*----------------------------------------------------------------------------*/
/*                             Private Definitions                            */
/*----------------------------------------------------------------------------*/
namespace
{

std::ofstream open_output_file(const std::string& filename)
{
    std::ofstream out(filename);
    if (!out.is_open()) {
        throw std::runtime_error("Failed to open file: " + filename);
    }
    out << std::fixed << std::setprecision(6);
    return out;
}

} /* unnamed namespace */
