/*-------------------------------- FILE INFO ---------------------------------*/
/* Filename           : optimizer.hpp                                         */
/*                                                                            */
/* Interface to maze building logic for micromouse simulations                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#ifndef MAZE_HPP_
#define MAZE_HPP_

/*----------------------------------------------------------------------------*/
/*                             Public Definitions                             */
/*----------------------------------------------------------------------------*/
namespace maze
{

constexpr double OFFICIAL_POST_SIZE {12.07}; /* 12.07mm */
constexpr double OFFICIAL_WALL_LENGTH_SIZE {166.37}; /* 166.37mm */
constexpr double OFFICIAL_WALL_WIDTH_SIZE {12.07}; /* 12.07mm */

} /* maze namespace */

/*----------------------------------------------------------------------------*/
/*                             Public Declarations                            */
/*----------------------------------------------------------------------------*/
namespace maze
{

geometry::RectangularHitbox create_post(const geometry::Point& center, 
            double horizontal_size_adjustment, double vertical_size_adjustment);

geometry::RectangularHitbox create_vertical_wall(const geometry::Point& center, 
            double horizontal_size_adjustment, double vertical_size_adjustment);

geometry::RectangularHitbox create_horizontal_wall(const geometry::Point& center, 
            double horizontal_size_adjustment, double vertical_size_adjustment);

} /* maze namespace */

#endif /* MAZE_HPP_ */
