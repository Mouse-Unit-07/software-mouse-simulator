/*-------------------------------- FILE INFO ---------------------------------*/
/* Filename           : front_wall_detection_optimizer.hpp                    */
/*                                                                            */
/* Interface to a micromouse simulation front_wall_detection_optimizer        */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#ifndef FRONT_WALL_DETECTION_OPTIMIZER_HPP_
#define FRONT_WALL_DETECTION_OPTIMIZER_HPP_

/*----------------------------------------------------------------------------*/
/*                             Public Definitions                             */
/*----------------------------------------------------------------------------*/
namespace front_wall_detection_optimizer
{

struct ParetoResult {
    std::vector<std::vector<double>> X;
    std::vector<std::vector<double>> F;
};

ParetoResult run_front_wall_detection_staged(std::size_t population, std::size_t generations,
                                             int simulations_per_fitness);
void write_pareto_to_file(const std::string& filename, const ParetoResult& result);

} /* front_wall_detection_optimizer namespace */

/*----------------------------------------------------------------------------*/
/*                             Public Declarations                            */
/*----------------------------------------------------------------------------*/
/* none */

#endif /* FRONT_WALL_DETECTION_OPTIMIZER_HPP_ */
