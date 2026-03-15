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

#include <cmath>
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
namespace geometry
{

RectangularHitbox::RectangularHitbox(const Point& center, double horizontal_size, double vertical_size)
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

bool RectangularHitbox::operator==(const RectangularHitbox& other) const noexcept
{
    constexpr double tolerance {1e-6};

    return (std::abs(horizontal_size - other.horizontal_size) <= tolerance)
        && (std::abs(vertical_size - other.vertical_size) <= tolerance)
        && (center == other.center) && (edge_1 == other.edge_1)
        && (edge_2 == other.edge_2) && (edge_3 == other.edge_3) 
        && (edge_4 == other.edge_4);
}

} /* geometry namespace */

/*----------------------------------------------------------------------------*/
/*                             Private Definitions                            */
/*----------------------------------------------------------------------------*/
/* none */
