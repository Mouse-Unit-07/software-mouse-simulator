/*-------------------------------- FILE INFO ---------------------------------*/
/* Filename           : maze.cpp                                              */
/*                                                                            */
/* Implementation for maze building logic for micromouse simulations          */
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
#include "maze.hpp"

/*----------------------------------------------------------------------------*/
/*                            Private Declarations                            */
/*----------------------------------------------------------------------------*/
namespace
{

geometry::RectangularHitbox create_post(const geometry::Point& center, 
            double horizontal_size_adjustment, double vertical_size_adjustment);

geometry::RectangularHitbox create_vertical_wall(const geometry::Point& center, 
            double horizontal_size_adjustment, double vertical_size_adjustment);

geometry::RectangularHitbox create_horizontal_wall(const geometry::Point& center, 
            double horizontal_size_adjustment, double vertical_size_adjustment);

}

/*----------------------------------------------------------------------------*/
/*                               Private Globals                              */
/*----------------------------------------------------------------------------*/
/* none */

/*----------------------------------------------------------------------------*/
/*                             Public Definitions                             */
/*----------------------------------------------------------------------------*/
namespace maze
{



} /* maze namespace */

/*----------------------------------------------------------------------------*/
/*                             Private Definitions                            */
/*----------------------------------------------------------------------------*/
namespace
{

constexpr double OFFICIAL_POST_SIZE {12.07}; /* 12.07mm */
constexpr double OFFICIAL_WALL_LENGTH_SIZE {166.37}; /* 166.37mm */
constexpr double OFFICIAL_WALL_WIDTH_SIZE {12.07}; /* 12.07mm */

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

}
