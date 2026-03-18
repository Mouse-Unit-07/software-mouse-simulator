/*-------------------------------- FILE INFO ---------------------------------*/
/* Filename           : rectangular_hitbox.hpp                                */
/*                                                                            */
/* Interface to rectangular_hitbox class                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#ifndef RECTANGULAR_HITBOX_HPP_
#define RECTANGULAR_HITBOX_HPP_

/*----------------------------------------------------------------------------*/
/*                             Public Definitions                             */
/*----------------------------------------------------------------------------*/
namespace geometry
{

class RectangularHitbox
{
public:
    Point center;
    Point top_right;
    Point top_left;
    Point bottom_left;
    Point bottom_right;
    double vertical_size;
    double horizontal_size;
    RectangularHitbox(const Point& center, double horizontal_size, double vertical_size);

    void translate(double dx, double dy) noexcept;
    void rotate(const Point& center, double angle_rad) noexcept;

    bool operator==(const RectangularHitbox& other) const noexcept;
    bool operator!=(const RectangularHitbox& other) const noexcept;
};

} /* geometry namespace */

/*----------------------------------------------------------------------------*/
/*                             Public Declarations                            */
/*----------------------------------------------------------------------------*/
/* none */

#endif /* RECTANGULAR_HITBOX_HPP_ */
