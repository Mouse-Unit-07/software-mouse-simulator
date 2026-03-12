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

#include "point.hpp"
#include "rectangular_hitbox.hpp"
#include "ray.hpp"
#include "interactions.hpp"

/*----------------------------------------------------------------------------*/
/*                            Private Declarations                            */
/*----------------------------------------------------------------------------*/
namespace
{

bool ray_intersects_line_segment(const geometry::Ray& ray, 
    const geometry::Point& a, const geometry::Point& b)
{
    double rdx = ray.front.x;
    double rdy = ray.front.y;

    double sdx = b.x - a.x;
    double sdy = b.y - a.y;

    double denom = rdx * sdy - rdy * sdx;

    if (denom == 0.0)
        return false;  // parallel

    double dx = a.x - ray.back.x;
    double dy = a.y - ray.back.y;

    double t = (dx * sdy - dy * sdx) / denom;
    double u = (dx * rdy - dy * rdx) / denom;

    return (t >= 0.0) && (u >= 0.0) && (u <= 1.0);
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

bool does_ray_intersect_hitbox(const Ray& ray, const RectangularHitbox& hitbox)
{

    return ray_intersects_line_segment(ray, hitbox.edge_1, hitbox.edge_2)
        || ray_intersects_line_segment(ray, hitbox.edge_2, hitbox.edge_3)
        || ray_intersects_line_segment(ray, hitbox.edge_3, hitbox.edge_4)
        || ray_intersects_line_segment(ray, hitbox.edge_4, hitbox.edge_1);
}

} /* geometry namespace */

/*----------------------------------------------------------------------------*/
/*                             Private Definitions                            */
/*----------------------------------------------------------------------------*/
namespace
{



}
