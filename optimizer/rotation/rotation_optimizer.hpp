/*-------------------------------- FILE INFO ---------------------------------*/
/* Filename           : rotation_optimizer.hpp                                */
/*                                                                            */
/* Interface to a micromouse simulation rotation_optimizer                    */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#ifndef ROTATION_OPTIMIZER_HPP_
#define ROTATION_OPTIMIZER_HPP_

/*----------------------------------------------------------------------------*/
/*                             Public Definitions                             */
/*----------------------------------------------------------------------------*/
namespace rotation_optimizer
{

struct ParetoResult {
    std::vector<std::vector<double>> X;
    std::vector<std::vector<double>> F;
};

ParetoResult run_rotation_pareto(std::size_t population, std::size_t generations,
                                 int simulations_per_fitness);
void write_rotation_pareto_to_file(const std::string& filename, const ParetoResult& result);

} /* rotation_optimizer namespace */

/*----------------------------------------------------------------------------*/
/*                             Public Declarations                            */
/*----------------------------------------------------------------------------*/
/* none */

#endif /* ROTATION_OPTIMIZER_HPP_ */
