/*
 * Encoder handler functions and definitions.
 *
 * Author: Tyler Holmes (tyler@holmesengineering.com)
 * Date: 1/2/2016
 */

#ifndef _MOTOR_CALC_H_
#define _MOTOR_CALC_H_

#include <stdint.h>
#include "motor_drivers.h"

/*******************************************************************************
 * Public Data Structures
 ******************************************************************************/

typedef enum {
	kPID_stop,
    kPID_velocity,
    kPID_distance
} PID_mode_t;


/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define MOTOR_CALC_UPDATE_RATE_MS 10


/*******************************************************************************
 * Public Function Definitions
 ******************************************************************************/


void motor_calc_init(void);
void motor_calc_parameters(void);

float   motor_calc_velocity_get(motor_select_t motor_desired);
int32_t motor_calc_distance_get(motor_select_t motor_desired);
void motor_calc_velocity_clear(motor_select_t motor_desired);
void motor_calc_distance_clear(motor_select_t motor_desired);

void motor_calc_PID_run(void);
void motor_calc_PID_setmode(PID_mode_t new_PID_mode);
void motor_calc_PID_set_target(motor_select_t motor_desired, PID_mode_t PID_mode, void * data);
bool motor_calc_PID_is_done(motor_select_t motor_desired, float err_tollerance);

#endif /*_MOTOR_CALC_H_*/
