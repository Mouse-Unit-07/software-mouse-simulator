/*-------------------------------- FILE INFO ---------------------------------*/
/* Filename           : ray.hpp                                               */
/*                                                                            */
/* Interface to ray class                                                     */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#ifndef RAY_HPP_
#define RAY_HPP_

/*----------------------------------------------------------------------------*/
/*                             Public Definitions                             */
/*----------------------------------------------------------------------------*/
namespace geometry
{

class Ray
{
    public:
        Point origin;
        Point direction;
        Ray(const Point& origin, double angle_rad);

        void translate(double dx, double dy) noexcept;
        void rotate(const Point& center, double angle_rad) noexcept;

        bool operator==(const Ray& other) const noexcept;
        bool operator!=(const Ray& other) const noexcept;
};

} /* geometry namespace */

/*----------------------------------------------------------------------------*/
/*                             Public Declarations                            */
/*----------------------------------------------------------------------------*/
/* none */

#endif /* RAY_HPP_ */
