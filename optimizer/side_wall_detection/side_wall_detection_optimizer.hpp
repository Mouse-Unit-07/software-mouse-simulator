/*-------------------------------- FILE INFO ---------------------------------*/
/* Filename           : side_wall_detection_optimizer.hpp                     */
/*                                                                            */
/* Interface to a micromouse simulation side_wall_detection_optimizer         */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#ifndef SIDE_WALL_DETECTION_OPTIMIZER_HPP_
#define SIDE_WALL_DETECTION_OPTIMIZER_HPP_

/*----------------------------------------------------------------------------*/
/*                             Public Definitions                             */
/*----------------------------------------------------------------------------*/
namespace side_wall_detection_optimizer
{

struct ParetoResult {
    std::vector<std::vector<double>> X;
    std::vector<std::vector<double>> F;
};

ParetoResult run_side_wall_detection_filter(std::size_t population, std::size_t generations,
                                            int simulations_per_fitness);
void write_pareto_to_file(const std::string& filename, const ParetoResult& result);

} /* side_wall_detection_optimizer namespace */

/*----------------------------------------------------------------------------*/
/*                             Public Declarations                            */
/*----------------------------------------------------------------------------*/
/* none */

#endif /* SIDE_WALL_DETECTION_OPTIMIZER_HPP_ */
