/*-------------------------------- FILE INFO ---------------------------------*/
/* Filename           : simulation_common.cpp                                 */
/*                                                                            */
/* Implementation for common micromouse simulation utilities                  */
/*                                                                            */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*                               Include Files                                */
/*----------------------------------------------------------------------------*/
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>
#include "simulation_common.hpp"

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
namespace simulation_common
{

std::string double_to_filename(double v, int precision)
{
    std::ostringstream oss{};

    double rounded{std::round(v)};
    if (std::abs(v - rounded) < 1e-9) {
        oss << static_cast<long long>(rounded);
    } else {
        oss << std::fixed << std::setprecision(precision + 6) << v;
    }

    std::string s{oss.str()};

    /* Handle sign early */
    bool negative{false};
    if (!s.empty() && s[0] == '-') {
        negative = true;
        s.erase(0, 1);
    }

    /* Split integer/fractional */
    size_t dot{s.find('.')};
    std::string int_part{(dot == std::string::npos) ? s : s.substr(0, dot)};
    std::string frac_part{(dot == std::string::npos) ? "" : s.substr(dot + 1)};

    /* Remove trailing zeros in fractional part */
    while (!frac_part.empty() && frac_part.back() == '0') {
        frac_part.pop_back();
    }

    std::string out{};

    if (int_part != "0") {
        /* normal case: integer part exists */
        out = int_part;
        if (!frac_part.empty()) {
            out += 'p' + frac_part.substr(0, precision);
        }
    } else {
        /* sub-1 case: strip leading zeros in fractional part */
        size_t first_non_zero{frac_part.find_first_not_of('0')};

        if (first_non_zero == std::string::npos) {
            out = "0";
        } else {
            out = "p" + frac_part.substr(first_non_zero, precision);
        }
    }

    if (negative) {
        out = "n" + out;
    }

    return out;
}

} /* simulation_common namespace */

/*----------------------------------------------------------------------------*/
/*                             Private Definitions                            */
/*----------------------------------------------------------------------------*/
/* none */
