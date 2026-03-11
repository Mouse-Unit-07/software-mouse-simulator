/*-------------------------------- FILE INFO ---------------------------------*/
/* Filename           : rectangular_hitbox.cpp                                */
/*                                                                            */
/* Implementation for rectangular_hitbox class                                */
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
RectangularHitbox::RectangularHitbox(Point center, double horizontal_size, double vertical_size)
    : center{center}, horizontal_size{horizontal_size}, vertical_size{vertical_size}
{
    edge_1 = Point{center.x + (horizontal_size / 2), center.y + (vertical_size / 2)};
    edge_2 = Point{center.x + (horizontal_size / 2), center.y - (vertical_size / 2)};
    edge_3 = Point{center.x - (horizontal_size / 2), center.y - (vertical_size / 2)};
    edge_4 = Point{center.x - (horizontal_size / 2), center.y + (vertical_size / 2)};
}

void RectangularHitbox::translate(double dx, double dy) noexcept
{
    center.translate(dx, dy);
    edge_1.translate(dx, dy);
    edge_2.translate(dx, dy);
    edge_3.translate(dx, dy);
    edge_4.translate(dx, dy);
}

void RectangularHitbox::rotate(const Point& center, double angle_rad) noexcept
{
    this->center.rotate(center, angle_rad);
    edge_1.rotate(center, angle_rad);
    edge_2.rotate(center, angle_rad);
    edge_3.rotate(center, angle_rad);
    edge_4.rotate(center, angle_rad);
}

/*----------------------------------------------------------------------------*/
/*                             Private Definitions                            */
/*----------------------------------------------------------------------------*/
/* none */
