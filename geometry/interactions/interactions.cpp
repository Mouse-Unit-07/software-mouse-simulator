/*-------------------------------- FILE INFO ---------------------------------*/
/* Filename           : interactions.cpp                                      */
/*                                                                            */
/* Implementation for interactions between geometrical classes                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*                               Include Files                                */
/*----------------------------------------------------------------------------*/
extern "C"
{

}

#include <cmath>
#include <optional>
#include "point.hpp"
#include "rectangular_hitbox.hpp"
#include "ray.hpp"
#include "interactions.hpp"

/*----------------------------------------------------------------------------*/
/*                            Private Declarations                            */
/*----------------------------------------------------------------------------*/
namespace
{

std::optional<double> ray_segment_distance(const geometry::Ray& ray, 
    const geometry::Point& a, const geometry::Point& b)
{
    double rdx = ray.direction.x;
    double rdy = ray.direction.y;

    double sdx = b.x - a.x;
    double sdy = b.y - a.y;

    double denom = rdx * sdy - rdy * sdx;

    if (std::abs(denom) < 1e-6)
        return std::nullopt;

    double dx = a.x - ray.origin.x;
    double dy = a.y - ray.origin.y;

    double t = (dx * sdy - dy * sdx) / denom;
    double u = (dx * rdy - dy * rdx) / denom;

    if (t >= 0.0 && u >= 0.0 && u <= 1.0)
        return t;

    return std::nullopt;
}

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

std::optional<double> ray_hitbox_distance(const Ray& ray,
    const RectangularHitbox& hitbox)
{
    std::optional<double> closest;

    auto update = [&](std::optional<double> d)
    {
        if (!d)
            return;

        if (!closest || *d < *closest)
            closest = d;
    };

    update(ray_segment_distance(ray, hitbox.edge_1, hitbox.edge_2));
    update(ray_segment_distance(ray, hitbox.edge_2, hitbox.edge_3));
    update(ray_segment_distance(ray, hitbox.edge_3, hitbox.edge_4));
    update(ray_segment_distance(ray, hitbox.edge_4, hitbox.edge_1));

    return closest;
}

} /* geometry namespace */

/*----------------------------------------------------------------------------*/
/*                             Private Definitions                            */
/*----------------------------------------------------------------------------*/
namespace
{



}
