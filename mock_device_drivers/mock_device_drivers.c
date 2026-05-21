/*-------------------------------- FILE INFO ---------------------------------*/
/* Filename           : mock_device_drivers.c                                 */
/*                                                                            */
/* Mock implementation for device drivers layer                               */
/*                                                                            */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*                               Include Files                                */
/*----------------------------------------------------------------------------*/
#include <math.h>
#include <stdint.h>
#include "infrared_sensor.h"
#include "magnetic_encoder.h"
#include "wheel_motor.h"
#include "mock_device_drivers.h"

/*----------------------------------------------------------------------------*/
/*                            Private Declarations                            */
/*----------------------------------------------------------------------------*/
uint32_t compute_ir_sensor_reading_from_distance_mm(double distance);

/*----------------------------------------------------------------------------*/
/*                               Private Globals                              */
/*----------------------------------------------------------------------------*/
static const double ADC_COUNTS_PER_VOLT = ((2.2 / (1.0 + 2.2) ) / 1.8) * 1024;

static const double PIECEWISE_BREAKPOINT_CM = 4.0;

/* short-range linear fit */
static const double LINEAR_SLOPE = -0.35220170;
static const double LINEAR_OFFSET = 2.50291193;

/* long-range inverse fit */
static const double INVERSE_NUMERATOR = 5.36005479;
static const double INVERSE_OFFSET = -0.13938373;

static const double MAX_MOTOR_RPM = 4900.0;
static const double ENCODER_EVENTS_PER_REVOLUTION = 60.8077;
static double MAX_ENCODER_TICKS_PER_SECOND =
    (MAX_MOTOR_RPM / 60.0) * (ENCODER_EVENTS_PER_REVOLUTION / 4);

static const double GEAR_RATIO = 13.0 / 44.0;
static const double WHEEL_DIAMETER_MM = 32.0;
static const double WHEEL_BASE_MM = 87.56;
static double WHEEL_CIRCUMFERENCE_MM = M_PI * WHEEL_DIAMETER_MM;

static double const ENCODER_TICKS_PER_REVOLUTION =
    (ENCODER_EVENTS_PER_REVOLUTION / 4) * (1 / GEAR_RATIO);

/* these two variables need to be accessible for movement simulation */
double ENCODER_TICKS_PER_MILLIMETER = ENCODER_TICKS_PER_REVOLUTION / (M_PI * WHEEL_DIAMETER_MM);
double ENCODER_TICKS_PER_ROTATION_ANGLE_RADIANS =
    (ENCODER_TICKS_PER_REVOLUTION * WHEEL_BASE_MM) / (2 * M_PI * WHEEL_DIAMETER_MM);

static uint32_t mock_ir_1_sensor_reading = 0u;
static uint32_t mock_ir_2_sensor_reading = 0u;
static uint32_t mock_ir_3_sensor_reading = 0u;
static uint32_t mock_ir_4_sensor_reading = 0u;

enum wheel_motor_direction
{
    BACKWARD_DIRECTION = -1,
    FORWARD_DIRECTION = 1
};

static uint8_t wheel_motor_1_speed = 0u;
static uint8_t wheel_motor_2_speed = 0u;
static enum wheel_motor_direction wheel_motor_1_direction = FORWARD_DIRECTION;
static enum wheel_motor_direction wheel_motor_2_direction = FORWARD_DIRECTION;

static double encoder_1_ticks = 0.0;
static double encoder_2_ticks = 0.0;

static double motor_speed_scale = 1.0;
static double motor_1_variance = 0.0;
static double motor_2_variance = 0.0;
static double motor_slip_factor = 1.0;
static double wheel_circumference_scale = 1.0;
static double wheel_base_scale = 1.0;

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

    encoder_1_ticks = 0.0;
    encoder_2_ticks = 0.0;

    motor_speed_scale = 1.0;
    motor_1_variance = 0.0;
    motor_2_variance = 0.0;
    motor_slip_factor = 1.0;
    wheel_circumference_scale = 1.0;
    wheel_base_scale = 1.0;
}

void update_encoder_1_ticks(double time_elapsed_sec)
{
    double encoder_ticks_per_second = MAX_ENCODER_TICKS_PER_SECOND * (wheel_motor_1_speed / 255.0)
                                      * motor_speed_scale * (1.0 + motor_1_variance)
                                      * motor_slip_factor;
    double change_in_ticks =
        (encoder_ticks_per_second * time_elapsed_sec) * wheel_motor_1_direction;

    encoder_1_ticks += change_in_ticks;
}

void update_encoder_2_ticks(double time_elapsed_sec)
{
    double encoder_ticks_per_second = MAX_ENCODER_TICKS_PER_SECOND * (wheel_motor_2_speed / 255.0)
                                      * motor_speed_scale * (1.0 + motor_2_variance)
                                      * motor_slip_factor;
    double change_in_ticks =
        (encoder_ticks_per_second * time_elapsed_sec) * wheel_motor_2_direction;

    encoder_2_ticks += change_in_ticks;
}

void update_ir_1_sensor_reading(double distance)
{
    mock_ir_1_sensor_reading = compute_ir_sensor_reading_from_distance_mm(distance);
}

void update_ir_2_sensor_reading(double distance)
{
    mock_ir_2_sensor_reading = compute_ir_sensor_reading_from_distance_mm(distance);
}

void update_ir_3_sensor_reading(double distance)
{
    mock_ir_3_sensor_reading = compute_ir_sensor_reading_from_distance_mm(distance);
}

void update_ir_4_sensor_reading(double distance)
{
    mock_ir_4_sensor_reading = compute_ir_sensor_reading_from_distance_mm(distance);
}

uint32_t scale_and_clamp_ir_sensor_reading(uint32_t reading, double scale)
{
    double scaled = (double)reading * scale;

    if (scaled <= 0.0) {
        return 0;
    }
    if (scaled >= 1024.0) {
        return 1024;
    }

    return (uint32_t)lround(scaled);
}

void set_motor_speed_scale(double scale)
{
    motor_speed_scale = scale;
}

void set_motor_1_variance(double variance)
{
    motor_1_variance = variance;
}

void set_motor_2_variance(double variance)
{
    motor_2_variance = variance;
}

void set_motor_slip_factor(double factor)
{
    motor_slip_factor = factor;
}

void set_wheel_circumference_scale(double scale)
{
    wheel_circumference_scale = scale;
}

void set_wheel_base_scale(double scale)
{
    wheel_base_scale = scale;
}

struct mouse_delta compute_mouse_delta(double current_mouse_angle, double time_elapsed_sec)
{
    double motor_1_rpm = (wheel_motor_1_speed / 255.0) * MAX_MOTOR_RPM * motor_speed_scale
                         * (1.0 + motor_1_variance) * wheel_motor_1_direction;

    double motor_2_rpm = (wheel_motor_2_speed / 255.0) * MAX_MOTOR_RPM * motor_speed_scale
                         * (1.0 + motor_2_variance) * wheel_motor_2_direction;

    double wheel_1_rpm = motor_1_rpm * GEAR_RATIO;
    double wheel_2_rpm = motor_2_rpm * GEAR_RATIO;

    double velocity_1 = wheel_1_rpm * (WHEEL_CIRCUMFERENCE_MM * wheel_circumference_scale) / 60.0;

    double velocity_2 = wheel_2_rpm * (WHEEL_CIRCUMFERENCE_MM * wheel_circumference_scale) / 60.0;

    double combined_velocity = (velocity_1 + velocity_2) / 2.0;

    double omega = (velocity_2 - velocity_1) / (WHEEL_BASE_MM * wheel_base_scale);

    double distance = combined_velocity * time_elapsed_sec * motor_slip_factor;

    double dtheta = omega * time_elapsed_sec * motor_slip_factor;

    struct mouse_delta delta = {0};
    delta.dx = distance * cos(current_mouse_angle);
    delta.dy = distance * sin(current_mouse_angle);
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
    return (int32_t)encoder_1_ticks;
}

int32_t get_encoder_2_ticks(void)
{
    return (int32_t)encoder_2_ticks;
}

void clear_1_encoder_ticks(void)
{
    encoder_1_ticks = 0.0;
}

void clear_2_encoder_ticks(void)
{
    encoder_2_ticks = 0.0;
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
uint32_t compute_ir_sensor_reading_from_distance_mm(double distance)
{
    /* below formula converts cm -> reading, so convert distance mm -> cm */
    double distance_cm = distance / 10.0;
    double voltage = 0.0;

    if (distance_cm <= PIECEWISE_BREAKPOINT_CM) {
        voltage = (LINEAR_SLOPE * distance_cm) + LINEAR_OFFSET;
    }
    else {
        voltage = (INVERSE_NUMERATOR / distance_cm) + INVERSE_OFFSET;
    }

    double value = voltage * ADC_COUNTS_PER_VOLT;

    if (value < 0.0) {
        value = 0.0;
    }

    if (value > 1024.0) {
        value = 1024.0;
    }

    return (uint32_t)lround(value);
}
