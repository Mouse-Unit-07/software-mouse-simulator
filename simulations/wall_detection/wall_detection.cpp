/*-------------------------------- FILE INFO ---------------------------------*/
/* Filename           : wall_detection.cpp                                    */
/*                                                                            */
/* Implementation for micromouse wall detection simulation and associated     */
/* config and results analysis helpers                                        */
/*                                                                            */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*                               Include Files                                */
/*----------------------------------------------------------------------------*/
extern "C"
{

#include <stdint.h>
#include <math.h>
#include "mock_device_drivers.h"
#include "infrared_sensor.h"

}

#include <vector>
#include <string>
#include <optional>
#include "point.hpp"
#include "ray.hpp"
#include "rectangular_hitbox.hpp"
#include "mouse.hpp"
#include "maze.hpp"
#include "wall_detection.hpp"

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
namespace wall_detection
{

Config build_config(const std::vector<double>& v)
{
    Config cfg{};

    int i {0};

    cfg.maze_size_scale = v[i++];
    cfg.ir_reading_scale = v[i++];
    cfg.mouse_angle = v[i++];
    cfg.horizontal_position_variance = v[i++];
    cfg.total_steps = v[i++];
    cfg.reading_threshold = v[i++];

    return cfg;
}

} /* wall_detection namespace */

/*----------------------------------------------------------------------------*/
/*                             Private Definitions                            */
/*----------------------------------------------------------------------------*/
/* none */
