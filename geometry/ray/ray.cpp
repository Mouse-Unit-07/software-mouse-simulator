/*-------------------------------- FILE INFO ---------------------------------*/
/* Filename           : ray.cpp                                               */
/*                                                                            */
/* Implementation for ray class                                               */
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
#include "ray.hpp"

/*----------------------------------------------------------------------------*/
/*                            Private Declarations                            */
/*----------------------------------------------------------------------------*/
namespace
{

bool check_ray_inequality(const geometry::Ray& r1, const geometry::Ray& r2);

}

/*----------------------------------------------------------------------------*/
/*                               Private Globals                              */
/*----------------------------------------------------------------------------*/
/* none */

/*----------------------------------------------------------------------------*/
/*                             Public Definitions                             */
/*----------------------------------------------------------------------------*/
namespace geometry
{

Ray::Ray(const Point& origin, double angle_rad)
    : origin{origin}
{
    double dir_x {std::cos(angle_rad)};
    double dir_y {std::sin(angle_rad)};
    direction.x = dir_x;
    direction.y = dir_y;
}

void Ray::translate(double dx, double dy) noexcept
{
    origin.translate(dx, dy);
    direction.translate(dx, dy);
}

void Ray::rotate(const Point& center, double angle_rad) noexcept
{
    origin.rotate(center, angle_rad);
    direction.rotate(center, angle_rad);
}

bool Ray::operator==(const Ray& other) const noexcept
{
    return check_ray_inequality(*this, other);
}

bool Ray::operator!=(const Ray& other) const noexcept
{
    return !check_ray_inequality(*this, other);
}

} /* geometry namespace */

/*----------------------------------------------------------------------------*/
/*                             Private Definitions                            */
/*----------------------------------------------------------------------------*/
namespace
{

bool check_ray_inequality(const geometry::Ray& r1, const geometry::Ray& r2)
{
    return (r1.origin == r2.origin) && (r1.direction == r2.direction);
}

}
