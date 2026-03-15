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
#include "magnetic_encoder.h"
#include "wheel_motor.h"
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

enum wheel_motor_direction
{
    BACKWARD_DIRECTION = -1,
    FORWARD_DIRECTION = 1
};

uint8_t wheel_motor_1_speed = 0u;
uint8_t wheel_motor_2_speed = 0u;
enum wheel_motor_direction wheel_motor_1_direction = FORWARD_DIRECTION;
enum wheel_motor_direction wheel_motor_2_direction = FORWARD_DIRECTION;

static const double MAX_MOTOR_RPM = 4900.0;
static const double ENCODER_EVENTS_PER_REVOLUTION = 60.8077;
static double MAX_ENCODER_TICKS_PER_SECOND = (MAX_MOTOR_RPM / 60.0) * (ENCODER_EVENTS_PER_REVOLUTION / 4);

int32_t encoder_1_ticks = 0;
int32_t encoder_2_ticks = 0;

static const double GEAR_RATIO = 13.0 / 44.0;
static const double WHEEL_DIAMETER_MM = 32.0;
static const double WHEEL_BASE_MM = 87.56;
static double WHEEL_CIRCUMFERENCE_MM = M_PI * WHEEL_DIAMETER_MM;

double motor_speed_scale = 1.0;
double motor_1_variance = 0.0;
double motor_2_variance = 0.0;
double motor_slip_factor = 1.0;

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

    wheel_motor_1_speed = 0u;
    wheel_motor_2_speed = 0u;
    wheel_motor_1_direction = FORWARD_DIRECTION;
    wheel_motor_2_direction = FORWARD_DIRECTION;

    encoder_1_ticks = 0;
    encoder_2_ticks = 0;

    motor_speed_scale = 1.0;
    motor_1_variance = 0.0;
    motor_2_variance = 0.0;
    motor_slip_factor = 1.0;
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

int32_t compute_new_encoder_1_ticks(double time_elapsed_sec)
{
    double encoder_ticks_per_second = MAX_ENCODER_TICKS_PER_SECOND * (wheel_motor_1_speed / 255.0);
    int32_t change_in_ticks = (int32_t)((encoder_ticks_per_second * time_elapsed_sec) * wheel_motor_1_direction);

    return encoder_1_ticks + change_in_ticks;
}

int32_t compute_new_encoder_2_ticks(double time_elapsed_sec)
{
    double encoder_ticks_per_second = MAX_ENCODER_TICKS_PER_SECOND * (wheel_motor_2_speed / 255.0);
    int32_t change_in_ticks = (int32_t)((encoder_ticks_per_second * time_elapsed_sec) * wheel_motor_2_direction);

    return encoder_2_ticks + change_in_ticks;
}

void set_encoder_1_ticks(int32_t ticks)
{
    encoder_1_ticks = ticks;
}

void set_encoder_2_ticks(int32_t ticks)
{
    encoder_2_ticks = ticks;
}

void set_motor_speed_scale(double speed_scale)
{
    motor_speed_scale = speed_scale;
}

void set_motor_1_variance(double variance)
{
    motor_1_variance = variance;
}

void set_motor_2_variance(double variance)
{
    motor_2_variance = variance;
}

void set_motor_slip_factor(double slip_factor)
{
    motor_slip_factor = slip_factor;
}

struct mouse_delta compute_mouse_delta(double current_mouse_angle, double time_elapsed_sec)
{
    double motor_1_rpm =
        (wheel_motor_1_speed / 255.0) *
        MAX_MOTOR_RPM *
        motor_speed_scale *
        (1.0 + motor_1_variance) *
        wheel_motor_1_direction;

    double motor_2_rpm =
        (wheel_motor_2_speed / 255.0) *
        MAX_MOTOR_RPM *
        motor_speed_scale *
        (1.0 + motor_2_variance) *
        wheel_motor_2_direction;

    double wheel_1_rpm = motor_1_rpm * GEAR_RATIO;
    double wheel_2_rpm = motor_2_rpm * GEAR_RATIO;

    double velocity_1 =
        wheel_1_rpm *
        WHEEL_CIRCUMFERENCE_MM /
        60.0;

    double velocity_2 =
        wheel_2_rpm *
        WHEEL_CIRCUMFERENCE_MM /
        60.0;

    double combined_velocity = (velocity_1 + velocity_2) / 2.0;

    double omega =
        (velocity_2 - velocity_1) /
        WHEEL_BASE_MM;

    double distance =
        combined_velocity *
        time_elapsed_sec *
        motor_slip_factor;

    double dtheta =
        omega *
        time_elapsed_sec *
        motor_slip_factor;

    struct mouse_delta delta = {0};

    delta.dx =
        distance * cos(current_mouse_angle);

    delta.dy =
        distance * sin(current_mouse_angle);

    delta.dtheta_rad = dtheta;

    return delta;
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
    return encoder_1_ticks;
}

int32_t get_encoder_2_ticks(void)
{
    return encoder_2_ticks;
}

void clear_1_encoder_ticks(void)
{
    encoder_1_ticks = 0;
}

void clear_2_encoder_ticks(void)
{
    encoder_2_ticks = 0;
}

/* -------------------------------------------------------------------------- */
/* wheel_motor mocks */
void set_wheel_motor_1_speed(uint8_t speed)
{
    wheel_motor_1_speed = speed;
}

void set_wheel_motor_2_speed(uint8_t speed)
{
    wheel_motor_2_speed = speed;
}

void set_wheel_motor_1_direction_forward(void)
{
    wheel_motor_1_direction = FORWARD_DIRECTION;
}

void set_wheel_motor_1_direction_backward(void)
{
    wheel_motor_1_direction = BACKWARD_DIRECTION;
}

void set_wheel_motor_2_direction_forward(void)
{
    wheel_motor_2_direction = FORWARD_DIRECTION;
}

void set_wheel_motor_2_direction_backward(void)
{
    wheel_motor_2_direction = BACKWARD_DIRECTION;
}

/*----------------------------------------------------------------------------*/
/*                             Private Definitions                            */
/*----------------------------------------------------------------------------*/
/* none */
