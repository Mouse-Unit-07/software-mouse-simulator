/*-------------------------------- FILE INFO ---------------------------------*/
/* Filename           : simulation_common.hpp                                 */
/*                                                                            */
/* Interface to common micromouse simulation utilities                        */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#ifndef SIMULATION_COMMON_HPP_
#define SIMULATION_COMMON_HPP_

/*----------------------------------------------------------------------------*/
/*                             Public Definitions                             */
/*----------------------------------------------------------------------------*/
/* none */

/*----------------------------------------------------------------------------*/
/*                             Public Declarations                            */
/*----------------------------------------------------------------------------*/
namespace simulation_common
{

std::string double_to_filename(double v, int precision = 3);

} /* simulation_common namespace */

#endif /* SIMULATION_COMMON_HPP_ */
