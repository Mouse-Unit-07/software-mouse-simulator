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
class Ray
{
    public:
        Point origin;
        Point direction;
        Ray(const Point& origin, const Point& direction);

        void translate(double dx, double dy) noexcept;
        void rotate(const Point& center, double angle_rad) noexcept;

        bool operator==(const Ray& other) const noexcept;
};

#endif /* RAY_HPP_ */
