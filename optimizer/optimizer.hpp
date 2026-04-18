/*-------------------------------- FILE INFO ---------------------------------*/
/* Filename           : optimizer.hpp                                         */
/*                                                                            */
/* Interface to a micromouse simulation optimizer                             */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#ifndef OPTIMIZER_HPP_
#define OPTIMIZER_HPP_

/*----------------------------------------------------------------------------*/
/*                             Public Definitions                             */
/*----------------------------------------------------------------------------*/
namespace optimizer
{

struct ParetoResult {
    std::vector<std::vector<double>> X;
    std::vector<std::vector<double>> F;
};

ParetoResult run_rotation_pareto(std::size_t population, std::size_t generations,
                                 int simulations_per_fitness);
void write_rotation_pareto_to_file(const std::string& filename, const ParetoResult& result);

ParetoResult run_move_forward_pareto(std::size_t population, std::size_t generations,
                                     int simulations_per_fitness, move_forward::WallMode mode);
ParetoResult run_move_forward_staged(size_t population, size_t gen_stage1, size_t gen_stage2,
                                     int sims_stage1, int sims_stage2, move_forward::WallMode mode);
void write_move_forward_pareto_to_file(const std::string& filename, const ParetoResult& result);

} /* optimizer namespace */

/*----------------------------------------------------------------------------*/
/*                             Public Declarations                            */
/*----------------------------------------------------------------------------*/
/* none */

#endif /* OPTIMIZER_HPP_ */
