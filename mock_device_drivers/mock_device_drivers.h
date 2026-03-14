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
struct displacement
{
    double dx;
    double dy;
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
struct displacement compute_mouse_position_change(int time_elapsed);
double compute_mouse_angle_change(int time_elapsed);

#endif /* MOCK_DEVICE_DRIVERS_HPP_ */
