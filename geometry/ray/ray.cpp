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

#include "point.hpp"
#include "ray.hpp"

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
Ray::Ray(const Point& back, const Point& front)
    : back{back}, front{front}
{
    /* no logic- only field init */
}

void Ray::translate(double dx, double dy) noexcept
{
    back.translate(dx, dy);
    front.translate(dx, dy);
}

void Ray::rotate(const Point& center, double angle_rad) noexcept
{
    back.rotate(center, angle_rad);
    front.rotate(center, angle_rad);
}

bool Ray::operator==(const Ray& other) const noexcept
{
    return (back == other.back) && (front == other.front);
}

/*----------------------------------------------------------------------------*/
/*                             Private Definitions                            */
/*----------------------------------------------------------------------------*/
/* none */
