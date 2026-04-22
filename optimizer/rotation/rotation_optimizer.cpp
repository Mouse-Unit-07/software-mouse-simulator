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

std::unordered_map<std::string, double> time_cache{};

std::vector<size_t> get_best_indices(const std::vector<PagmoVec>& F, size_t keep_n);

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

struct Stage1Objectives {
    double collision{};
    double timeout{};
    double angle{};

    PagmoVec to_vec() const
    {
        return {collision, timeout, angle};
    }

    static Stage1Objectives from_vec(const PagmoVec& v)
    {
        return {v.at(0), v.at(1), v.at(2)};
    }
};

struct Stage2Objectives {
    double collision{};
    double timeout{};
    double angle{};
    double translation{};

    PagmoVec to_vec() const
    {
        return {collision, timeout, angle, translation};
    }

    static Stage2Objectives from_vec(const PagmoVec& v)
    {
        return {v.at(0), v.at(1), v.at(2), v.at(3)};
    }
};

class RotationFeasibilityUDP {
public:
    RotationFeasibilityUDP() = default;

    RotationFeasibilityUDP(int sims) : sims_(sims)
    {
        /* no additional logic */
    }

    PagmoVec fitness(const PagmoVec& x) const
    {
        const auto control{rotation::decode_control(x)};

        double collision{0.0};
        double timeout{0.0};
        double angle{0.0};

        for (int i{0}; i < sims_; ++i) {
            rotation::Config cfg{control, rotation::generate_random_environment()};
            const auto r{rotation::run_simulation(cfg)};

            collision += r.collision ? 1.0 : 0.0;
            timeout += r.timeout ? 1.0 : 0.0;
            angle += r.final_angle_error;
        }

        Stage1Objectives obj;
        obj.collision = collision / sims_;
        obj.timeout = timeout / sims_;
        obj.angle = angle / sims_;

        return obj.to_vec();
    }

    std::pair<PagmoVec, PagmoVec> get_bounds() const
    {
        return rotation::get_control_bounds();
    }

    pagmo::vector_double::size_type get_nobj() const
    {
        return 3;
    }

private:
    int sims_{100};
};

class RotationUDP {
public:
    RotationUDP() = default;

    RotationUDP(int sims) : sims_(sims)
    {
        /* no additional logic */
    }

    PagmoVec fitness(const PagmoVec& x) const
    {
        const auto control{rotation::decode_control(x)};

        double collision{0.0};
        double timeout{0.0};
        double angle{0.0};
        double translation{0.0};
        double total_time{0.0};

        for (int i{0}; i < sims_; ++i) {
            rotation::Config cfg{control, rotation::generate_random_environment()};
            const auto r{rotation::run_simulation(cfg)};

            collision += r.collision ? 1.0 : 0.0;
            timeout += r.timeout ? 1.0 : 0.0;
            angle += r.final_angle_error;
            translation += r.final_translation;
            total_time += r.total_time;
        }

        /* cache metadata */
        time_cache[optimizer_common::control_to_key(x)] = total_time / sims_;

        Stage2Objectives obj;
        obj.collision = collision / sims_;
        obj.timeout = timeout / sims_;
        obj.angle = angle / sims_;
        obj.translation = translation / sims_;

        return obj.to_vec();
    }

    std::pair<PagmoVec, PagmoVec> get_bounds() const
    {
        return rotation::get_control_bounds();
    }

    pagmo::vector_double::size_type get_nobj() const
    {
        return 4;
    }

private:
    int sims_{100};
};

ParetoResult run_rotation_staged(std::size_t population, std::size_t gen_stage1,
                                 std::size_t gen_stage2, int sims_stage1, int sims_stage2)
{
    pagmo::algorithm algo{pagmo::nsga2{}};

    /* Stage 1 */
    pagmo::problem prob1{RotationFeasibilityUDP{sims_stage1}};
    pagmo::population pop1{prob1, population};

    for (size_t i{0}; i < gen_stage1; ++i) {
        pop1 = algo.evolve(pop1);
    }
    ParetoResult stage1{pop1.get_x(), pop1.get_f()};

    auto best_indices{get_best_indices(stage1.F, population)};

    std::vector<PagmoVec> seeds{};
    for (auto idx : best_indices) {
        seeds.push_back(stage1.X.at(idx));
    }

    /* Stage 2 */
    pagmo::problem prob2{RotationUDP{sims_stage2}};
    pagmo::population pop2{prob2, 0};

    for (auto& x : seeds) {
        pop2.push_back(x);
    }

    size_t i{0};
    while (pop2.size() < population) {
        pop2.push_back(seeds.at(i % seeds.size()));
        ++i;
    }

    for (size_t i{0}; i < gen_stage2; ++i) {
        pop2 = algo.evolve(pop2);
    }

    return {pop2.get_x(), pop2.get_f()};
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
    constexpr int W_SC{6};

    constexpr int W_COLL{12};
    constexpr int W_TO{12};
    constexpr int W_ANGLE{12};
    constexpr int W_TRANS{12};
    constexpr int W_TIME{12};

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
        << std::setw(W_COLL) << "collision"
        << std::setw(W_TO) << "timeout"
        << std::setw(W_ANGLE) << "angle"
        << std::setw(W_TRANS) << "translation"
        << std::setw(W_TIME) << "time"
        << "\n";

    out << std::string(120, '-') << "\n";

    for (size_t i{0}; i < result.X.size(); ++i) {
        const auto& x{result.X.at(i)};
        const auto& f{result.F.at(i)};
        const auto ctrl{rotation::decode_control(x)};
        const auto obj{Stage2Objectives::from_vec(f)};

        double time{0.0};
        auto it{time_cache.find(optimizer_common::control_to_key(x))};
        if (it != time_cache.end()) {
            time = it->second;
        }

        out << std::left
            << std::setw(W_IDX) << i
            << std::setw(W_SPD) << static_cast<int>(ctrl.motor_speed)
            << std::setw(W_KP_V) << ctrl.kp_velocity
            << std::setw(W_KD_V) << ctrl.kd_velocity
            << std::setw(W_KP_A) << ctrl.kp_angle
            << std::setw(W_KD_A) << ctrl.kd_angle
            << std::setw(W_SC) << ctrl.pid_scale
            << " | "
            << std::setw(W_COLL) << obj.collision
            << std::setw(W_TO) << obj.timeout
            << std::setw(W_ANGLE) << obj.angle
            << std::setw(W_TRANS) << obj.translation
            << std::setw(W_TIME) << time
            << "\n";
    }
}

} /* rotation_optimizer namespace */

/*----------------------------------------------------------------------------*/
/*                             Private Definitions                            */
/*----------------------------------------------------------------------------*/
namespace
{

std::vector<size_t> get_best_indices(const std::vector<PagmoVec>& F, size_t keep_n)
{
    std::vector<size_t> idx(F.size());
    std::iota(idx.begin(), idx.end(), 0);

    constexpr double FLOAT_TOLERANCE{1e-3};

    std::sort(idx.begin(), idx.end(), [&](size_t a, size_t b) {
        const auto A{rotation_optimizer::Stage1Objectives::from_vec(F.at(a))};
        const auto B{rotation_optimizer::Stage1Objectives::from_vec(F.at(b))};

        if (std::abs(A.collision - B.collision) > FLOAT_TOLERANCE)
            return A.collision < B.collision;

        if (std::abs(A.timeout - B.timeout) > FLOAT_TOLERANCE)
            return A.timeout < B.timeout;

        return A.angle < B.angle;
    });

    idx.resize(std::min(keep_n, idx.size()));
    return idx;
}

} /* unnamed namespace */
