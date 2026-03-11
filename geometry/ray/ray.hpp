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
/* none */

/*----------------------------------------------------------------------------*/
/*                             Public Declarations                            */
/*----------------------------------------------------------------------------*/
namespace geometry
{

class Ray
{
    public:
        Point back;
        Point front;
        Ray(const Point& back, const Point& front);

        void translate(double dx, double dy) noexcept;
        void rotate(const Point& center, double angle_rad) noexcept;

        bool operator==(const Ray& other) const noexcept;
};

} /* geometry namespace */

#endif /* RAY_HPP_ */
