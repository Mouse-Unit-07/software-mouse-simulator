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

int32_t compute_new_encoder_1_ticks(double time_elapsed_sec);
int32_t compute_new_encoder_2_ticks(double time_elapsed_sec);
void set_encoder_1_ticks(int32_t ticks);
void set_encoder_2_ticks(int32_t ticks);
uint32_t compute_ir_sensor_reading_from_distance_mm(double distance);
void set_ir_1_sensor_reading(uint32_t reading);
void set_ir_2_sensor_reading(uint32_t reading);
void set_ir_3_sensor_reading(uint32_t reading);
void set_ir_4_sensor_reading(uint32_t reading);
void set_motor_speed_scale(double scale);
void set_motor_1_variance(double variance); /* pass value from -1.0 to 1.0 */
void set_motor_2_variance(double variance); /* pass value from -1.0 to 1.0 */
void set_motor_slip_factor(double factor);
void set_wheel_circumference_scale(double scale);
void set_wheel_base_scale(double scale);
struct mouse_delta compute_mouse_delta(double current_mouse_angle, double time_elapsed_sec);

#endif /* MOCK_DEVICE_DRIVERS_HPP_ */
