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

 /*! Initializes the `motor_calc` data structures.
  *
  * @Note This should be called first before all the others.
  *
  * @param[in] current_tick (uint32_t): The current timer tick (in miliseconds)
  */
void motor_calc_init(uint32_t current_tick);

/*! Function to be called periodically to update position and velocity values.
 *
 * @Note This should be called periodically to keep motor parameters up to date.
 *
 * @param[in] left_encoder_counts (int32_t): The current raw value for the left encoder.
 * @param[in] right_encoder_counts (int32_t): The current raw value for the left encoder.
 * @param[in] current_tick (uint32_t): The current timer tick when calling this
                function in miliseconds
 */
void motor_calc_parameters(int32_t left_encoder_counts, int32_t right_encoder_counts, uint32_t current_tick);

/*! Returns the requested motors current velocity (in meters / second).
 *
 * @param[in] motor_desired (motor_select_t): Which motor you want the velocity from
 * @param[out] motor_velocity (float): The current velocity from the selected motor
                (in meters / second).
 */
float   motor_calc_velocity_get(motor_select_t motor_desired);

/*! Returns the requested motors total traveled distance (in milimeters).
 *
 * @param[in] motor_desired (motor_select_t): Which motor you want the distance from
 * @param[out] motor_velocity (int32_t): The total traveled distance (in milimeters).
 */
int32_t motor_calc_distance_get(motor_select_t motor_desired);

/*! Clears the requested motors current velocity.
 *
 * @param[in] motor_desired (motor_select_t): Which motors velocity you want to clear.
 */
void motor_calc_velocity_clear(motor_select_t motor_desired);

/*! Clears the requested motors distance travelled.
 *
 * @param[in] motor_desired (motor_select_t): Which motors distance you want to clear.
 */
void motor_calc_distance_clear(motor_select_t motor_desired);

/*! Runs PID calculations for distance or velocity compliance. Should be called
 *      periodically to ensure you don't crash the robot.
 *
 * @param[in] err_output_l_ptr (float *): A pointer to where the resulting error
        term is stored for the left motor.
 * @param[in] err_output_r_ptr (float *): A pointer to where the resulting error
        term is stored for the right motor.
 */
void motor_calc_PID_run(float *err_output_l_ptr, float *err_output_r_ptr);

/*! Sets the target value for the given motor.
 *
 * @Note you need to set the PID mode correctly before setting the target value.
 *
 * @param[in] motor_desired (motor_select_t): The motor you wish to update.
 * @param[in] data (void *): Pointer to the value you wish to set as the target.
        Should be float for velocity and int32_t for distance.
 */
void motor_calc_PID_set_target(motor_select_t motor_desired, void * data);

/*! Function to decide whether or not the robot has made it to it's target. This
        is mostly useful for distance based PID modes.
 *
 * @param[in] motor_desired (motor_select_t): The motor you wish to check.
 * @param[in] err_tollerance (float): The amount of margin you want to give when
        determining whether or not the motor has reached the target.
 * @param[out] is_complete (bool): true if it has reached the target within
        `err_tollerance`.
 */
bool motor_calc_PID_is_done(motor_select_t motor_desired, float err_tollerance);

/*! Sets the PID mode to be used (either distance, velocity, or none) and Resets
 *      the internal state variables.
 *
 * @param[in] new_PID_mode (PID_mode_t): The new mode to be set.
 */
void motor_calc_PID_setmode(PID_mode_t new_PID_mode);

/*! Gets the current PID mode (either distance, velocity, or none).
 *
 * @param[out] current_PID_mode (PID_mode_t): The mode being used.
 */
PID_mode_t motor_calc_PID_getmode(void);

#endif /*_MOTOR_CALC_H_*/
