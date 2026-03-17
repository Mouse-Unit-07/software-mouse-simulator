/*-------------------------------- FILE INFO ---------------------------------*/
/* Filename           : mouse.cpp                                             */
/*                                                                            */
/* Implementation of mouse class for micromouse simulation                    */
/*                                                                            */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*                               Include Files                                */
/*----------------------------------------------------------------------------*/
extern "C"
{

}

#include <cmath>
#include "point.hpp"
#include "ray.hpp"
#include "rectangular_hitbox.hpp"
#include "mouse.hpp"

/*----------------------------------------------------------------------------*/
/*                            Private Declarations                            */
/*----------------------------------------------------------------------------*/
namespace
{

bool check_mouse_equality(const mouse::Mouse& m1, const mouse::Mouse& m2);

} /* unnamed namespace */

/*----------------------------------------------------------------------------*/
/*                               Private Globals                              */
/*----------------------------------------------------------------------------*/
namespace mouse
{

struct Displacement
{
    double dx;
    double dy;
};

constexpr double MOUSE_HITBOX_HORIZONTAL_MM {94.16};
constexpr double MOUSE_HITBOX_VERTICAL_MM {88.90};
constexpr Displacement IR_1_OFFSET_FROM_MOUSE_CENTER {-27.29, 25.24};
constexpr Displacement IR_2_OFFSET_FROM_MOUSE_CENTER {-17.45, 38.76};
constexpr Displacement IR_3_OFFSET_FROM_MOUSE_CENTER {17.45, 38.76};
constexpr Displacement IR_4_OFFSET_FROM_MOUSE_CENTER {27.29, 25.24};

} /* mouse namespace */

/*----------------------------------------------------------------------------*/
/*                             Public Definitions                             */
/*----------------------------------------------------------------------------*/
namespace mouse
{

Mouse::Mouse()
    : hitbox{geometry::Point{0.0, 0.0}, MOUSE_HITBOX_HORIZONTAL_MM, MOUSE_HITBOX_VERTICAL_MM},
    ir_1_sensor{geometry::Point{IR_1_OFFSET_FROM_MOUSE_CENTER.dx, IR_1_OFFSET_FROM_MOUSE_CENTER.dy}, M_PI / 2.0},
    ir_2_sensor{geometry::Point{IR_2_OFFSET_FROM_MOUSE_CENTER.dx, IR_2_OFFSET_FROM_MOUSE_CENTER.dy}, M_PI * (3.0 / 4.0)},
    ir_3_sensor{geometry::Point{IR_3_OFFSET_FROM_MOUSE_CENTER.dx, IR_3_OFFSET_FROM_MOUSE_CENTER.dy}, M_PI / 4.0},
    ir_4_sensor{geometry::Point{IR_4_OFFSET_FROM_MOUSE_CENTER.dx, IR_4_OFFSET_FROM_MOUSE_CENTER.dy}, M_PI / 2.0}
{
    /* no additional logic */
}

void Mouse::translate(double dx, double dy) noexcept
{
    hitbox.translate(dx, dy);
    ir_1_sensor.translate(dx, dy);
    ir_2_sensor.translate(dx, dy);
    ir_3_sensor.translate(dx, dy);
    ir_4_sensor.translate(dx, dy);
}

void Mouse::rotate(double angle_rad) noexcept
{
    hitbox.rotate(hitbox.center, angle_rad);
    ir_1_sensor.rotate(hitbox.center, angle_rad);
    ir_2_sensor.rotate(hitbox.center, angle_rad);
    ir_3_sensor.rotate(hitbox.center, angle_rad);
    ir_4_sensor.rotate(hitbox.center, angle_rad);
}

bool Mouse::operator==(const Mouse& other) const noexcept
{
    return check_mouse_equality(*this, other);
}

bool Mouse::operator!=(const Mouse& other) const noexcept
{
    return !check_mouse_equality(*this, other);
}

} /* mouse namespace */

/*----------------------------------------------------------------------------*/
/*                             Private Definitions                            */
/*----------------------------------------------------------------------------*/
namespace
{

bool check_mouse_equality(const mouse::Mouse& m1, const mouse::Mouse& m2)
{
    return (m1.hitbox == m2.hitbox)
        && (m1.ir_1_sensor == m2.ir_1_sensor)
        && (m1.ir_2_sensor == m2.ir_2_sensor)
        && (m1.ir_3_sensor == m2.ir_3_sensor)
        && (m1.ir_4_sensor == m2.ir_4_sensor);
}

} /* unnamed namespace */
