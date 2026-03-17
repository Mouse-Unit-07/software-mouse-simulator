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
    top_right = Point{center.x + (horizontal_size / 2), center.y + (vertical_size / 2)};
    top_left = Point{center.x - (horizontal_size / 2), center.y + (vertical_size / 2)};
    bottom_left = Point{center.x - (horizontal_size / 2), center.y - (vertical_size / 2)};
    bottom_right = Point{center.x + (horizontal_size / 2), center.y - (vertical_size / 2)};
}

void RectangularHitbox::translate(double dx, double dy) noexcept
{
    center.translate(dx, dy);
    top_right.translate(dx, dy);
    top_left.translate(dx, dy);
    bottom_left.translate(dx, dy);
    bottom_right.translate(dx, dy);
}

void RectangularHitbox::rotate(const Point& center, double angle_rad) noexcept
{
    this->center.rotate(center, angle_rad);
    top_right.rotate(center, angle_rad);
    top_left.rotate(center, angle_rad);
    bottom_left.rotate(center, angle_rad);
    bottom_right.rotate(center, angle_rad);
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
        && (h1.center == h2.center) && (h1.top_right == h2.top_right)
        && (h1.top_left == h2.top_left) && (h1.bottom_left == h2.bottom_left) 
        && (h1.bottom_left == h2.bottom_left);
}

} /* unnamed namespace */
