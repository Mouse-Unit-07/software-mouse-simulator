/*-------------------------------- FILE INFO ---------------------------------*/
/* Filename           : move_forward_optimizer.hpp                            */
/*                                                                            */
/* Interface to a micromouse simulation move_forward_optimizer                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#ifndef MOVE_FORWARD_OPTIMIZER_HPP_
#define MOVE_FORWARD_OPTIMIZER_HPP_

/*----------------------------------------------------------------------------*/
/*                             Public Definitions                             */
/*----------------------------------------------------------------------------*/
namespace move_forward_optimizer
{

struct ParetoResult {
    std::vector<std::vector<double>> X{};
    std::vector<std::vector<double>> F{};
};

ParetoResult run_move_forward_staged(size_t population, size_t gen_stage1, size_t gen_stage2,
                                     int sims_stage1, int sims_stage2, move_forward::WallMode mode);
void write_move_forward_pareto_to_file(const std::string &filename, const ParetoResult &result);

} /* move_forward_optimizer namespace */

/*----------------------------------------------------------------------------*/
/*                             Public Declarations                            */
/*----------------------------------------------------------------------------*/
/* none */

#endif /* MOVE_FORWARD_OPTIMIZER_HPP_ */
