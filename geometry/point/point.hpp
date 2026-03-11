/*-------------------------------- FILE INFO ---------------------------------*/
/* Filename           : point.hpp                                             */
/*                                                                            */
/* Interface to point class                                                   */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#ifndef POINT_HPP_
#define POINT_HPP_

/*----------------------------------------------------------------------------*/
/*                             Public Definitions                             */
/*----------------------------------------------------------------------------*/
/* none */

/*----------------------------------------------------------------------------*/
/*                             Public Declarations                            */
/*----------------------------------------------------------------------------*/
class Point
{
    public:
        double x {0};
        double y {0};
        Point() = default;
        Point(double x, double y);

        void translate(double dx, double dy) noexcept;
        void rotate(const Point& rotation_center, double angle_rad) noexcept;

        bool operator==(const Point& other) const noexcept;
};

#endif /* POINT_HPP_ */
