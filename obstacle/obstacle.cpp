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

Post::Post(const geometry::Point& center, 
        double horizontal_size_adjustment, double vertical_size_adjustment)
    : hitbox{center, OFFICIAL_POST_SIZE + horizontal_size_adjustment, OFFICIAL_POST_SIZE + vertical_size_adjustment}
{
    /* no additional logic */
}

void Post::translate(double dx, double dy) noexcept
{
    hitbox.translate(dx, dy);
}

void Post::rotate(const geometry::Point& center, double angle_rad) noexcept
{
    hitbox.rotate(center, angle_rad);
}

} /* obstacle namespace */

/*----------------------------------------------------------------------------*/
/*                             Private Definitions                            */
/*----------------------------------------------------------------------------*/
/* none */
