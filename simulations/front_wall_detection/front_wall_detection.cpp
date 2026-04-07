/*-------------------------------- FILE INFO ---------------------------------*/
/* Filename           : front_wall_detection.cpp                              */
/*                                                                            */
/* Implementation for micromouse front wall detection simulation and          */
/* associated config and results analysis helpers                             */
/*                                                                            */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*                               Include Files                                */
/*----------------------------------------------------------------------------*/
extern "C"
{

}

#include <cstdint>
#include <vector>
#include <string>
#include <map>
#include "simulation_common.hpp"
#include "front_wall_detection.hpp"

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
namespace front_wall_detection
{

bool ConfigSweeper::next()
{
    if (!initialized_) {
        sweeper.init_sizes({
            ir_reading_scale.size(),
            mouse_angle.size(),
            horizontal_position_variance.size(),
            vertical_position_variance.size(),
            reading_threshold.size()
        });

        initialized_ = true;
    }
    return sweeper.next();
}

Config ConfigSweeper::value() const
{
    const auto& idx {sweeper.get_indices()};
    int i{0};

    Config cfg{};

    cfg.ir_reading_scale = ir_reading_scale.at(idx.at(i++));
    cfg.mouse_angle = mouse_angle.at(idx.at(i++));
    cfg.horizontal_position_variance = horizontal_position_variance.at(idx.at(i++));
    cfg.vertical_position_variance = vertical_position_variance.at(idx.at(i++));
    cfg.reading_threshold = reading_threshold.at(idx.at(i++));

    return cfg;
}

} /* front_wall_detection namespace */

/*----------------------------------------------------------------------------*/
/*                             Private Definitions                            */
/*----------------------------------------------------------------------------*/
/* none */
