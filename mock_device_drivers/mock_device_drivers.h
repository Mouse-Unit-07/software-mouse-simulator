/*-------------------------------- FILE INFO ---------------------------------*/
/* Filename           : mock_device_drivers.h                                 */
/*                                                                            */
/* Mock interface to device drivers layer                                     */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#ifndef MOCK_DEVICE_DRIVERS_HPP_
#define MOCK_DEVICE_DRIVERS_HPP_

/*----------------------------------------------------------------------------*/
/*                             Public Definitions                             */
/*----------------------------------------------------------------------------*/
struct mouse_delta
{
    double dx;
    double dy;
    double dtheta_rad;
};

/*----------------------------------------------------------------------------*/
/*                             Public Declarations                            */
/*----------------------------------------------------------------------------*/
void reset_mock_device_drivers(void);

void update_encoder_1_ticks(double time_elapsed_sec);
void update_encoder_2_ticks(double time_elapsed_sec);
void update_ir_1_sensor_reading(double distance_mm);
void update_ir_2_sensor_reading(double distance_mm);
void update_ir_3_sensor_reading(double distance_mm);
void update_ir_4_sensor_reading(double distance_mm);
void set_motor_speed_scale(double scale);
void set_motor_1_variance(double variance); /* pass value from -1.0 to 1.0 */
void set_motor_2_variance(double variance); /* pass value from -1.0 to 1.0 */
void set_motor_slip_factor(double factor);
void set_wheel_circumference_scale(double scale);
void set_wheel_base_scale(double scale);
struct mouse_delta compute_mouse_delta(double current_mouse_angle, double time_elapsed_sec);

#endif /* MOCK_DEVICE_DRIVERS_HPP_ */
