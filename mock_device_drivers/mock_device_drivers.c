/*-------------------------------- FILE INFO ---------------------------------*/
/* Filename           : mock_device_drivers.c                                 */
/*                                                                            */
/* Mock implementation for device drivers layer                               */
/*                                                                            */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*                               Include Files                                */
/*----------------------------------------------------------------------------*/
#include <stdint.h>
#include <math.h>
#include "infrared_sensor.h"
#include "mock_device_drivers.h"

/*----------------------------------------------------------------------------*/
/*                            Private Declarations                            */
/*----------------------------------------------------------------------------*/
/* none */

/*----------------------------------------------------------------------------*/
/*                               Private Globals                              */
/*----------------------------------------------------------------------------*/
/* constant portion of equation to translate distance to IR sensor reading */
static const double CONSTANT_IR_SCALE = ((2.71272 * (2.2 / 3.2)) / 1.8) * 1024.0;

uint32_t mock_ir_1_sensor_reading = 0u;
uint32_t mock_ir_2_sensor_reading = 0u;
uint32_t mock_ir_3_sensor_reading = 0u;
uint32_t mock_ir_4_sensor_reading = 0u;

/*----------------------------------------------------------------------------*/
/*                             Public Definitions                             */
/*----------------------------------------------------------------------------*/
/* mock helpers */
void reset_mock_device_drivers(void)
{
    mock_ir_1_sensor_reading = 0u;
    mock_ir_2_sensor_reading = 0u;
    mock_ir_3_sensor_reading = 0u;
    mock_ir_4_sensor_reading = 0u;
}

uint32_t compute_ir_sensor_reading_from_distance_mm(double distance)
{
    double value = CONSTANT_IR_SCALE * pow(0.858585, distance / 100.0);

    if (value < 0.0)
        value = 0.0;

    if (value > 1024.0)
        value = 1024.0;

    return (uint32_t)value;
}

void set_ir_1_sensor_reading(uint32_t reading)
{
    mock_ir_1_sensor_reading = reading;
}

void set_ir_2_sensor_reading(uint32_t reading)
{
    mock_ir_2_sensor_reading = reading;
}

void set_ir_3_sensor_reading(uint32_t reading)
{
    mock_ir_3_sensor_reading = reading;
}

void set_ir_4_sensor_reading(uint32_t reading)
{
    mock_ir_4_sensor_reading = reading;
}

int32_t compute_new_encoder_1_ticks(int time_elapsed)
{
    return 0;
}

int32_t compute_new_encoder_2_ticks(int time_elapsed)
{
    return 0;
}

void set_encoder_1_ticks(int32_t ticks)
{

}

void set_encoder_2_ticks(int32_t ticks)
{

}

struct displacement compute_mouse_position_change(int time_elapsed)
{
    struct displacement val = {0};
    return val;
}

double compute_mouse_angle_change(int time_elapsed)
{
    return 0.0;
}

/* -------------------------------------------------------------------------- */
/* infrared_sensor mocks */
uint32_t read_ir_1_sensor(void)
{
    return mock_ir_1_sensor_reading;
}

uint32_t read_ir_2_sensor(void)
{
    return mock_ir_2_sensor_reading;
}

uint32_t read_ir_3_sensor(void)
{
    return mock_ir_3_sensor_reading;
}

uint32_t read_ir_4_sensor(void)
{
    return mock_ir_4_sensor_reading;
}

/* -------------------------------------------------------------------------- */
/* magnetic_encoder mocks */
int32_t get_encoder_1_ticks(void)
{
    return 0;
}

int32_t get_encoder_2_ticks(void)
{
    return 0;
}

void clear_1_encoder_ticks(void)
{

}

void clear_2_encoder_ticks(void)
{

}

/* -------------------------------------------------------------------------- */
/* wheel_motor mocks */
void set_wheel_motor_1_speed(uint8_t speed)
{

}

void set_wheel_motor_2_speed(uint8_t speed)
{

}

void set_wheel_motor_1_direction_forward(void)
{

}

void set_wheel_motor_1_direction_backward(void)
{

}

void set_wheel_motor_2_direction_forward(void)
{

}

void set_wheel_motor_2_direction_backward(void)
{

}

/*----------------------------------------------------------------------------*/
/*                             Private Definitions                            */
/*----------------------------------------------------------------------------*/
/* none */
