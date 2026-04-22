/*-------------------------------- FILE INFO ---------------------------------*/
/* Filename           : optimizer_common.cpp                                  */
/*                                                                            */
/* Implementation of a micromouse simulation optimizer_common                 */
/*                                                                            */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*                               Include Files                                */
/*----------------------------------------------------------------------------*/
#include <fstream>
#include <iomanip>
#include <sstream>
#include <vector>
#include "optimizer_common.hpp"

/*----------------------------------------------------------------------------*/
/*                            Private Declarations                            */
/*----------------------------------------------------------------------------*/
/* none */

/*----------------------------------------------------------------------------*/
/*                               Private Globals                              */
/*----------------------------------------------------------------------------*/
/* none */

/*----------------------------------------------------------------------------*/
/*                             Public Definitions                             */
/*----------------------------------------------------------------------------*/
namespace optimizer_common
{

std::ofstream open_output_file(const std::string& filename)
{
    std::ofstream out(filename);
    if (!out.is_open()) {
        throw std::runtime_error("Failed to open file: " + filename);
    }
    out << std::fixed << std::setprecision(6);
    return out;
}

std::string control_to_key(const std::vector<double>& x)
{
    std::ostringstream oss{};
    oss << std::fixed << std::setprecision(6);
    for (double v : x) {
        oss << v << ",";
    }
    return oss.str();
}

} /* optimizer_common namespace */

/*----------------------------------------------------------------------------*/
/*                             Private Definitions                            */
/*----------------------------------------------------------------------------*/
/* none */
