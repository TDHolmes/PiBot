/*
 * Encoder handler functions and definitions.
 *
 * Author: Tyler Holmes (tyler@holmesengineering.com)
 * Date: 1/2/2016
 */

#ifndef _MOTOR_CALC_H_
#define _MOTOR_CALC_H_

#include <stdint.h>
#include <stdbool.h>

#include "constants.h"


/*******************************************************************************
 * Public Function Definitions
 ******************************************************************************/


void motor_calc_init(uint32_t current_tick);
void motor_calc_parameters(int32_t left_encoder_counts, int32_t right_encoder_counts, uint32_t current_tick);

float   motor_calc_velocity_get(motor_select_t motor_desired);
int32_t motor_calc_distance_get(motor_select_t motor_desired);
void motor_calc_velocity_clear(motor_select_t motor_desired);
void motor_calc_distance_clear(motor_select_t motor_desired);

void motor_calc_PID_run(float *err_output_l_ptr, float *err_output_r_ptr);
void motor_calc_PID_setmode(PID_mode_t new_PID_mode);
PID_mode_t motor_calc_PID_getmode(void);
void motor_calc_PID_set_target(motor_select_t motor_desired, PID_mode_t PID_mode, void * data);
bool motor_calc_PID_is_done(motor_select_t motor_desired, float err_tollerance);

#endif /*_MOTOR_CALC_H_*/
