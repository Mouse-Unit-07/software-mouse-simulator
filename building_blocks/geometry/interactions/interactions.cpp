/*-------------------------------- FILE INFO ---------------------------------*/
/* Filename           : interactions.cpp                                      */
/*                                                                            */
/* Implementation for interactions between geometrical classes                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*                               Include Files                                */
/*----------------------------------------------------------------------------*/
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

using namespace geometry;

std::optional<double> ray_segment_distance(const Ray& ray, const Point& a, const Point& b);

double compute_dot_product(const Point& a, const Point& b);

Point compute_vector_between_points(const Point& a, const Point& b);

Point compute_perpendicular_vector(const Point& e);

void project_hitbox_onto_axis(const RectangularHitbox& box, const Point& axis, double& min,
                              double& max);

bool are_hitboxes_separated_on_axis(const RectangularHitbox& a, const RectangularHitbox& b,
                                    const Point& axis);

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

std::optional<double> compute_ray_hitbox_distance(const Ray& ray, const RectangularHitbox& hitbox)
{
    std::optional<double> closest{};

    auto update = [&](std::optional<double> d) {
        if (!d) {
            return;
        }

        if (!closest || (*d < *closest)) {
            closest = d;
        }
    };

    update(ray_segment_distance(ray, hitbox.top_right, hitbox.top_left));
    update(ray_segment_distance(ray, hitbox.top_left, hitbox.bottom_left));
    update(ray_segment_distance(ray, hitbox.bottom_left, hitbox.bottom_right));
    update(ray_segment_distance(ray, hitbox.bottom_right, hitbox.top_right));

    return closest;
}

bool do_hitboxes_overlap(const RectangularHitbox& a, const RectangularHitbox& b)
{
    Point axes[4]{
        compute_perpendicular_vector(compute_vector_between_points(a.top_right, a.top_left)),
        compute_perpendicular_vector(compute_vector_between_points(a.top_left, a.bottom_left)),
        compute_perpendicular_vector(compute_vector_between_points(b.top_right, b.top_left)),
        compute_perpendicular_vector(compute_vector_between_points(b.top_left, b.bottom_left))};

    for (const auto& axis : axes) {
        if (are_hitboxes_separated_on_axis(a, b, axis)) {
            return false;
        }
    }

    return true;
}

} /* geometry namespace */

/*----------------------------------------------------------------------------*/
/*                             Private Definitions                            */
/*----------------------------------------------------------------------------*/
namespace
{

using namespace geometry;

std::optional<double> ray_segment_distance(const Ray& ray, const Point& a, const Point& b)
{
    double rdx{ray.direction.x};
    double rdy{ray.direction.y};

    double sdx{b.x - a.x};
    double sdy{b.y - a.y};

    double denom{(rdx * sdy) - (rdy * sdx)};

    if (std::abs(denom) < 1e-6) {
        return std::nullopt;
    }

    double dx{a.x - ray.origin.x};
    double dy{a.y - ray.origin.y};

    double t{((dx * sdy) - (dy * sdx)) / denom};
    double u{((dx * rdy) - (dy * rdx)) / denom};

    if ((t >= 0.0) && (u >= 0.0) && (u <= 1.0)) {
        return t;
    }

    return std::nullopt;
}

double compute_dot_product(const Point& a, const Point& b)
{
    return (a.x * b.x) + (a.y * b.y);
}

Point compute_vector_between_points(const Point& a, const Point& b)
{
    return {b.x - a.x, b.y - a.y};
}

Point compute_perpendicular_vector(const Point& e)
{
    return {-e.y, e.x};
}

void project_hitbox_onto_axis(const RectangularHitbox& box, const Point& axis, double& min,
                              double& max)
{
    const Point *pts[4]{&box.top_right, &box.top_left, &box.bottom_left, &box.bottom_right};

    min = max = compute_dot_product(*pts[0], axis);

    for (int i{1}; i < 4; ++i) {
        double p{compute_dot_product(*pts[i], axis)};
        min = std::min(min, p);
        max = std::max(max, p);
    }
}

bool are_hitboxes_separated_on_axis(const RectangularHitbox& a, const RectangularHitbox& b,
                                    const Point& axis)
{
    double minA{};
    double maxA{};
    double minB{};
    double maxB{};

    project_hitbox_onto_axis(a, axis, minA, maxA);
    project_hitbox_onto_axis(b, axis, minB, maxB);

    return maxA < minB || maxB < minA;
}

} /* unnamed namespace */
