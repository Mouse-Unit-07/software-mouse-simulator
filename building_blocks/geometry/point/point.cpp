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
namespace
{

using namespace geometry;

bool check_point_equality(const Point& p1, const Point& p2);

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

Point::Point(double x, double y)
    : x{x}, y{y}
{
    /* no additional logic */
}

void Point::translate(double dx, double dy) noexcept
{
    x += dx;
    y += dy;
}

void Point::rotate(const Point& center, double angle_rad) noexcept
{
    const double sine_of_angle{std::sin(angle_rad)};
    const double cosine_of_angle{std::cos(angle_rad)};

    const double x_relative_to_center{x - center.x};
    const double y_relative_to_center{y - center.y};

    const double rotated_x_relative{
        (x_relative_to_center * cosine_of_angle)
        - (y_relative_to_center * sine_of_angle)
    };

    const double rotated_y_relative{
        (x_relative_to_center * sine_of_angle)
        + (y_relative_to_center * cosine_of_angle)
    };

    x = center.x + rotated_x_relative;
    y = center.y + rotated_y_relative;
}

bool Point::operator==(const Point& other) const noexcept
{
    return check_point_equality(*this, other);
}

bool Point::operator!=(const Point& other) const noexcept
{
    return !check_point_equality(*this, other);
}

} /* geometry namespace */

/*----------------------------------------------------------------------------*/
/*                             Private Definitions                            */
/*----------------------------------------------------------------------------*/
namespace
{

using namespace geometry;

bool check_point_equality(const Point& p1, const Point& p2)
{
    constexpr double tolerance{1e-6};

    return (std::abs(p1.x - p2.x) <= tolerance)
        && (std::abs(p1.y - p2.y) <= tolerance);
}

} /* unnamed namespace */
