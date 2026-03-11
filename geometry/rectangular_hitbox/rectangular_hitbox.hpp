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
/* none */

/*----------------------------------------------------------------------------*/
/*                             Public Declarations                            */
/*----------------------------------------------------------------------------*/
class RectangularHitbox
{
    public:
        Point center;
        Point edge_1;
        Point edge_2;
        Point edge_3;
        Point edge_4;
        double vertical_size;
        double horizontal_size;
        RectangularHitbox(Point center, double horizontal_size, double vertical_size);

        void translate(double dx, double dy) noexcept;
        void rotate(const Point& center, double angle_rad) noexcept;
};

#endif /* RECTANGULAR_HITBOX_HPP_ */
