/*-------------------------------- FILE INFO ---------------------------------*/
/* Filename           : mouse.hpp                                             */
/*                                                                            */
/* Interface to mouse class for micromouse simulations                       */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#ifndef MOUSE_HPP_
#define MOUSE_HPP_

/*----------------------------------------------------------------------------*/
/*                             Public Definitions                             */
/*----------------------------------------------------------------------------*/
/* none */

/*----------------------------------------------------------------------------*/
/*                             Public Declarations                            */
/*----------------------------------------------------------------------------*/
namespace mouse
{

class Mouse
{
    public:
        geometry::RectangularHitbox hitbox;
        geometry::Ray ir_1_sensor;
        geometry::Ray ir_2_sensor;
        geometry::Ray ir_3_sensor;
        geometry::Ray ir_4_sensor;
        Mouse();

        void translate(double dx, double dy) noexcept;
        void rotate(double angle_rad) noexcept;

        bool operator==(const Mouse& other) const noexcept;
        bool operator!=(const Mouse& other) const noexcept;
};

} /* mouse namespace */

#endif /* MOUSE_HPP_ */
