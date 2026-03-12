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

} /* obstacle namespace */

/*----------------------------------------------------------------------------*/
/*                             Private Definitions                            */
/*----------------------------------------------------------------------------*/
/* none */
