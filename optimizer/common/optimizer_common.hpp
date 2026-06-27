/*-------------------------------- FILE INFO ---------------------------------*/
/* Filename           : optimizer_common.hpp                                  */
/*                                                                            */
/* Interface to a micromouse simulation optimizer_common                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#ifndef OPTIMIZER_COMMON_HPP_
#define OPTIMIZER_COMMON_HPP_

/*----------------------------------------------------------------------------*/
/*                             Public Definitions                             */
/*----------------------------------------------------------------------------*/
namespace optimizer_common
{

std::ofstream open_output_file(const std::string &filename);
std::string control_to_key(const std::vector<double> &x);

} /* optimizer_common namespace */

/*----------------------------------------------------------------------------*/
/*                             Public Declarations                            */
/*----------------------------------------------------------------------------*/
/* none */

#endif /* OPTIMIZER_COMMON_HPP_ */
