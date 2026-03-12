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
    const geometry::Point& a, const geometry::Point& b);

double compute_dot_product(const geometry::Point& a, const geometry::Point& b);
geometry::Point compute_vector_between_points(const geometry::Point& a, const geometry::Point& b);
geometry::Point compute_perpendicular_vector(const geometry::Point& e);
void project_hitbox_onto_axis(const geometry::RectangularHitbox& box,
             const geometry::Point& axis,
             double& min,
             double& max);
bool are_hitboxes_separated_on_axis(const geometry::RectangularHitbox& a,
               const geometry::RectangularHitbox& b,
               const geometry::Point& axis);
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

bool do_hitboxes_overlap(const RectangularHitbox& a, const RectangularHitbox& b)
{
    Point axes[4] =
    {
        compute_perpendicular_vector(compute_vector_between_points(a.edge_1, a.edge_2)),
        compute_perpendicular_vector(compute_vector_between_points(a.edge_2, a.edge_3)),
        compute_perpendicular_vector(compute_vector_between_points(b.edge_1, b.edge_2)),
        compute_perpendicular_vector(compute_vector_between_points(b.edge_2, b.edge_3))
    };

    for (const auto& axis : axes)
    {
        if (are_hitboxes_separated_on_axis(a, b, axis))
            return false;
    }

    return true;
}

} /* geometry namespace */

/*----------------------------------------------------------------------------*/
/*                             Private Definitions                            */
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

double compute_dot_product(const geometry::Point& a, const geometry::Point& b)
{
    return a.x * b.x + a.y * b.y;
}

geometry::Point compute_vector_between_points(const geometry::Point& a, const geometry::Point& b)
{
    return {b.x - a.x, b.y - a.y};
}

geometry::Point compute_perpendicular_vector(const geometry::Point& e)
{
    return {-e.y, e.x};
}

void project_hitbox_onto_axis(const geometry::RectangularHitbox& box,
             const geometry::Point& axis,
             double& min,
             double& max)
{
    const geometry::Point* pts[4] =
    {
        &box.edge_1,
        &box.edge_2,
        &box.edge_3,
        &box.edge_4
    };

    min = max = compute_dot_product(*pts[0], axis);

    for (int i = 1; i < 4; ++i)
    {
        double p = compute_dot_product(*pts[i], axis);
        min = std::min(min, p);
        max = std::max(max, p);
    }
}

bool are_hitboxes_separated_on_axis(const geometry::RectangularHitbox& a,
               const geometry::RectangularHitbox& b,
               const geometry::Point& axis)
{
    double minA, maxA;
    double minB, maxB;

    project_hitbox_onto_axis(a, axis, minA, maxA);
    project_hitbox_onto_axis(b, axis, minB, maxB);

    return maxA < minB || maxB < minA;
}

}
