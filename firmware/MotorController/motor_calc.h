/*
 * Encoder handler functions and definitions.
 *
 * Author: Tyler Holmes (tyler@holmesengineering.com)
 * Date: 1/2/2016
 */

#ifndef _MOTOR_CALC_H_
#define _MOTOR_CALC_H_


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


#endif /*_MOTOR_CALC_H_*/
