/*-------------------------------- FILE INFO ---------------------------------*/
/* Filename           : move_forward_optimizer.cpp                            */
/*                                                                            */
/* Implementation of a micromouse simulation move_forward_optimizer           */
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
#include "move_forward.hpp"
#include "move_forward_optimizer.hpp"

/*----------------------------------------------------------------------------*/
/*                            Private Declarations                            */
/*----------------------------------------------------------------------------*/
namespace
{

using PagmoVec = pagmo::vector_double;

std::unordered_map<std::string, double> time_cache;

std::vector<size_t> get_best_feasible_indices(const std::vector<PagmoVec>& F, size_t keep_n);

} /* unnamed namespace */

/*----------------------------------------------------------------------------*/
/*                               Private Globals                              */
/*----------------------------------------------------------------------------*/
/* none */

/*----------------------------------------------------------------------------*/
/*                             Public Definitions                             */
/*----------------------------------------------------------------------------*/
namespace move_forward_optimizer
{

struct Stage1Objectives {
    double collision{};
    double horizontal{};
    double timeout{};

    PagmoVec to_vec() const
    {
        return {collision, horizontal, timeout};
    }

    static Stage1Objectives from_vec(const PagmoVec& v)
    {
        return {
            v.at(0),
            v.at(1),
            v.at(2)
        };
    }
};

struct Stage2Objectives {
    double collision{};
    double horizontal{};
    double timeout{};
    double vertical{};
    double angle{};

    PagmoVec to_vec() const
    {
        return {collision, horizontal, timeout, vertical, angle};
    }

    static Stage2Objectives from_vec(const PagmoVec& v)
    {
        return {
            v.at(0),
            v.at(1),
            v.at(2),
            v.at(3),
            v.at(4)
        };
    }
};

class MoveForwardFeasibilityUDP {
public:
    MoveForwardFeasibilityUDP() = default;

    MoveForwardFeasibilityUDP(int sims, move_forward::WallMode mode) : sims_(sims), mode_(mode)
    {
        /* no additional logic */
    }

    PagmoVec fitness(const PagmoVec& x) const
    {
        const auto control{move_forward::decode_control(x)};

        double collision{0.0};
        double horizontal{0.0};
        double timeout{0.0};

        for (int i{0}; i < sims_; ++i) {
            move_forward::Config cfg{control, move_forward::generate_random_environment()};

            const auto r{move_forward::run_simulation(cfg, mode_)};

            collision += r.collision ? 1.0 : 0.0;
            horizontal += r.final_horizontal_translation;
            timeout += r.timeout ? 1.0 : 0.0;
        }

        Stage1Objectives obj;
        obj.collision = collision / sims_;
        obj.horizontal = horizontal / sims_;
        obj.timeout = timeout / sims_;

        return obj.to_vec();
    }

    std::pair<PagmoVec, PagmoVec> get_bounds() const
    {
        return move_forward::get_control_bounds();
    }

    pagmo::vector_double::size_type get_nobj() const
    {
        return 3;
    }

private:
    int sims_{100};
    move_forward::WallMode mode_;
};

class MoveForwardUDP {
public:
    MoveForwardUDP() = default;

    MoveForwardUDP(int simulations_per_fitness, move_forward::WallMode mode)
        : sims_(simulations_per_fitness), mode_(mode)
    {
        /* no additional logic */
    }

    PagmoVec fitness(const PagmoVec& x) const
    {
        const auto control{move_forward::decode_control(x)};

        double collision{0.0};
        double horizontal{0.0};
        double timeout{0.0};
        double vertical{0.0};
        double angle{0.0};
        double total_time{0.0};

        for (int i{0}; i < sims_; ++i) {
            move_forward::Config cfg{control, move_forward::generate_random_environment()};

            const auto r{move_forward::run_simulation(cfg, mode_)};

            collision += r.collision ? 1.0 : 0.0;
            horizontal += r.final_horizontal_translation;
            timeout += r.timeout ? 1.0 : 0.0;
            vertical += r.final_vertical_translation;
            angle += r.final_angle_error;
            total_time += r.total_time;
        }

        /* metadata */
        time_cache[optimizer_common::control_to_key(x)] = total_time / sims_;

        Stage2Objectives obj;
        obj.collision = collision / sims_;
        obj.horizontal = horizontal / sims_;
        obj.timeout = timeout / sims_;
        obj.vertical = vertical / sims_;
        obj.angle = angle / sims_;

        return obj.to_vec();
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
    move_forward::WallMode mode_;
};

ParetoResult run_move_forward_staged(size_t population, size_t gen_stage1, size_t gen_stage2,
                                     int sims_stage1, int sims_stage2, move_forward::WallMode mode)
{
    /* Stage 1: Feasibility */
    pagmo::problem prob1{MoveForwardFeasibilityUDP{sims_stage1, mode}};
    pagmo::algorithm algo{pagmo::nsga2{}};
    pagmo::population pop1{prob1, population};

    for (std::size_t i{0}; i < gen_stage1; ++i) {
        pop1 = algo.evolve(pop1);
    }
    ParetoResult stage1{pop1.get_x(), pop1.get_f()};

    /* Extract non-dominated solutions */
    auto best_indices{get_best_feasible_indices(stage1.F, population)};

    std::vector<PagmoVec> seeds;
    for (auto idx : best_indices) {
        seeds.push_back(stage1.X[idx]);
    }

    /* Stage 2: Full optimization */
    pagmo::problem prob{MoveForwardUDP{sims_stage2, mode}};
    pagmo::population pop(prob, 0);

    /* Inject seeds */
    for (auto& x : seeds) {
        pop.push_back(x);
    }

    /* Fill remaining population */
    size_t i{0};
    while (pop.size() < population) {
        pop.push_back(seeds[i % seeds.size()]);
        ++i;
    }

    /* Evolve */
    for (size_t i{0}; i < gen_stage2; ++i) {
        pop = algo.evolve(pop);
    }

    return {pop.get_x(), pop.get_f()};
}

void write_move_forward_pareto_to_file(const std::string& filename, const ParetoResult& result)
{
    auto out{optimizer_common::open_output_file(filename)};

    /* column widths */
    constexpr int W_IDX{4};
    constexpr int W_TGT{6};
    constexpr int W_SPD{6};
    constexpr int W_KP_V{6};
    constexpr int W_KD_V{6};
    constexpr int W_KP_A{10};
    constexpr int W_KD_A{6};
    constexpr int W_SC{10};
    constexpr int W_KP_IR{8};
    constexpr int W_KD_IR{8};

    constexpr int W_COLL{12};
    constexpr int W_HORIZ{12};
    constexpr int W_TO{12};
    constexpr int W_VERT{12};
    constexpr int W_ANGLE{12};
    constexpr int W_TIME{12};

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
        << std::setw(W_COLL)  << "collision"
        << std::setw(W_HORIZ) << "horiz"
        << std::setw(W_TO)    << "timeout"
        << std::setw(W_VERT)  << "vert"
        << std::setw(W_ANGLE) << "angle"
        << std::setw(W_TIME)  << "time"
        << "\n";

    out << std::string(150, '-') << "\n";

    /* rows */
    for (size_t i{0}; i < result.X.size(); ++i) {
        const auto& x{result.X.at(i)};
        const auto& f{result.F.at(i)};
        const auto ctrl{move_forward::decode_control(x)};
        const auto obj{Stage2Objectives::from_vec(f)};
        
        const std::string key{optimizer_common::control_to_key(x)};
        double time{0.0};
        auto it{time_cache.find(key)};
        if (it != time_cache.end()) {
            time = it->second;
        }

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
            << std::setw(W_COLL)  << obj.collision
            << std::setw(W_HORIZ) << obj.horizontal
            << std::setw(W_TO)    << obj.timeout
            << std::setw(W_VERT)  << obj.vertical
            << std::setw(W_ANGLE) << obj.angle
            << std::setw(W_TIME)  << time
            << "\n";
    }
}

} /* move_forward_optimizer namespace */

/*----------------------------------------------------------------------------*/
/*                             Private Definitions                            */
/*----------------------------------------------------------------------------*/
namespace
{

using namespace move_forward_optimizer;

std::vector<size_t> get_best_feasible_indices(const std::vector<PagmoVec>& F, size_t keep_n)
{
    std::vector<size_t> indices(F.size());
    std::iota(indices.begin(), indices.end(), 0);

    constexpr double FLOAT_TOLERANCE{1e-3};

    std::sort(indices.begin(), indices.end(), [&](size_t a, size_t b) {
        const auto A{Stage1Objectives::from_vec(F.at(a))};
        const auto B{Stage1Objectives::from_vec(F.at(b))};

        if (std::abs(A.collision - B.collision) > FLOAT_TOLERANCE) {
            return A.collision < B.collision;
        }

        if (std::abs(A.horizontal - B.horizontal) > FLOAT_TOLERANCE) {
            return A.horizontal < B.horizontal;
        }

        return A.timeout < B.timeout;
    });

    indices.resize(std::min(keep_n, indices.size()));
    return indices;
}

} /* unnamed namespace */
