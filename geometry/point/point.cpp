/*-------------------------------- FILE INFO ---------------------------------*/
/* Filename           : point.cpp                                             */
/*                                                                            */
/* Implementation for point class                                             */
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
Point::Point(double x, double y)
    : x{x}, y{y}
{
    /* no constructor logic- only field init */
}

void Point::translate(double dx, double dy) noexcept
{
    x += dx;
    y += dy;
}

void Point::rotate(const Point& center, double angle_rad) noexcept
{
    const double sine_of_angle   = std::sin(angle_rad);
    const double cosine_of_angle = std::cos(angle_rad);

    const double x_relative_to_center = x - center.x;
    const double y_relative_to_center = y - center.y;

    const double rotated_x_relative =
        x_relative_to_center * cosine_of_angle -
        y_relative_to_center * sine_of_angle;

    const double rotated_y_relative =
        x_relative_to_center * sine_of_angle +
        y_relative_to_center * cosine_of_angle;

    x = center.x + rotated_x_relative;
    y = center.y + rotated_y_relative;
}

bool Point::operator==(const Point& other) const noexcept
{
    constexpr double tolerance = 1e-6;

    return (std::abs(x - other.x) <= tolerance)
        && (std::abs(y - other.y) <= tolerance);
}

/*----------------------------------------------------------------------------*/
/*                             Private Definitions                            */
/*----------------------------------------------------------------------------*/
/* none */
