/*-------------------------------- FILE INFO ---------------------------------*/
/* Filename           : obstacle.cpp                                          */
/*                                                                            */
/* Implementation for obstacle class for micromouse simulations               */
/*                                                                            */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*                               Include Files                                */
/*----------------------------------------------------------------------------*/
extern "C"
{

}

#include "point.hpp"
#include "rectangular_hitbox.hpp"
#include "obstacle.hpp"

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
namespace obstacle
{

geometry::RectangularHitbox create_post(const geometry::Point& center, 
            double horizontal_size_adjustment, double vertical_size_adjustment)
{
    return geometry::RectangularHitbox{
        center,
        OFFICIAL_POST_SIZE + horizontal_size_adjustment,
        OFFICIAL_POST_SIZE + vertical_size_adjustment
    };
}

geometry::RectangularHitbox create_vertical_wall(const geometry::Point& center, 
            double horizontal_size_adjustment, double vertical_size_adjustment)
{
    return geometry::RectangularHitbox{
        center,
        OFFICIAL_WALL_WIDTH_SIZE + horizontal_size_adjustment,
        OFFICIAL_WALL_LENGTH_SIZE + vertical_size_adjustment
    };
}

geometry::RectangularHitbox create_horizontal_wall(const geometry::Point& center, 
            double horizontal_size_adjustment, double vertical_size_adjustment)
{
    return geometry::RectangularHitbox{
        center,
        OFFICIAL_WALL_LENGTH_SIZE + horizontal_size_adjustment,
        OFFICIAL_WALL_WIDTH_SIZE + vertical_size_adjustment
    };
}

} /* obstacle namespace */

/*----------------------------------------------------------------------------*/
/*                             Private Definitions                            */
/*----------------------------------------------------------------------------*/
/* none */
