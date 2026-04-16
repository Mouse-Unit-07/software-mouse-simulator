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

ParetoResult run_rotation_pareto(std::size_t population, std::size_t generations);
void write_pareto_to_file(const std::string& filename, const ParetoResult& result);

} /* optimizer namespace */

/*----------------------------------------------------------------------------*/
/*                             Public Declarations                            */
/*----------------------------------------------------------------------------*/
/* none */

#endif /* OPTIMIZER_HPP_ */
