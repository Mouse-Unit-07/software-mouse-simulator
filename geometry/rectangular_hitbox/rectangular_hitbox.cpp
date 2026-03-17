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
namespace
{

bool check_hitbox_equality(const geometry::RectangularHitbox& h1, const geometry::RectangularHitbox& h2);

} /* unnamed namespace */

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
    return check_hitbox_equality(*this, other);
}

bool RectangularHitbox::operator!=(const RectangularHitbox& other) const noexcept
{
    return !check_hitbox_equality(*this, other);
}

} /* geometry namespace */

/*----------------------------------------------------------------------------*/
/*                             Private Definitions                            */
/*----------------------------------------------------------------------------*/
namespace
{

bool check_hitbox_equality(const geometry::RectangularHitbox& h1, const geometry::RectangularHitbox& h2)
{
    constexpr double tolerance {1e-6};

    return (std::abs(h1.horizontal_size - h2.horizontal_size) <= tolerance)
        && (std::abs(h1.vertical_size - h2.vertical_size) <= tolerance)
        && (h1.center == h2.center) && (h1.edge_1 == h2.edge_1)
        && (h1.edge_2 == h2.edge_2) && (h1.edge_3 == h2.edge_3) 
        && (h1.edge_4 == h2.edge_4);
}

} /* unnamed namespace */
