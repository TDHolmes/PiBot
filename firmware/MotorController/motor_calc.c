/*
 * Position and velocity calculations. To be called periodically.
 *
 *
 * Author: Tyler Holmes (tyler@holmesengineering.com)
 * Date: 1/2/2016
 */

// libraries
#include <stdbool.h>
#include <stdint.h>

#include "motor_calc.h"
#include "constants.h"


//! Data structure to keep track of important motor statistics.
typedef struct {
    // current and previous velocities
    float   vel[2];
    float   vel_prev[2];

    // current and previous positions
    int32_t pos[2];
    int32_t pos_prev[2];

    // previous encoder and time values for delta calculations
    int32_t enc_prev[2];
    int32_t time_prev;
} motor_stats_t;


//! Data structure to keep track of relevent PID variables.
typedef struct {
    PID_mode_t mode;
    float      vel_target[2];
    int32_t    pos_target[2];
    // keep track of integral error over time as well as previous error for derivative.
    float      err_prev[2];
    float      err_integral[2];
} motor_PID_t;


motor_stats_t motor_stats;
motor_PID_t  PID_admin;


void motor_calc_init(uint32_t current_tick)
{
    // initialize the motor stat terms
    motor_stats.vel[kMotor_Left]  = 0;
    motor_stats.vel[kMotor_Right] = 0;
    motor_stats.vel_prev[kMotor_Left]  = 0;
    motor_stats.vel_prev[kMotor_Right] = 0;
    motor_stats.pos[kMotor_Left]  = 0;
    motor_stats.pos[kMotor_Right] = 0;
    motor_stats.pos_prev[kMotor_Left]  = 0;
    motor_stats.pos_prev[kMotor_Right] = 0;
    motor_stats.enc_prev[kMotor_Left]  = 0;
    motor_stats.enc_prev[kMotor_Right] = 0;
    motor_stats.time_prev = current_tick;

    // initialize the PID terms
    PID_admin.mode = kPID_stop;
    PID_admin.vel_target[kMotor_Left]  = 0;
    PID_admin.vel_target[kMotor_Right] = 0;
    PID_admin.pos_target[kMotor_Left]  = 0;
    PID_admin.pos_target[kMotor_Right] = 0;
    PID_admin.err_prev[kMotor_Left]  = 0;
    PID_admin.err_prev[kMotor_Right] = 0;
    PID_admin.err_integral[kMotor_Left]  = 0;
    PID_admin.err_integral[kMotor_Right] = 0;

}


// function to be called periodically to update position and velocity.
void motor_calc_parameters(int32_t left_encoder_counts, int32_t right_encoder_counts, uint32_t current_tick)
{
    int32_t enc_l_delta, enc_r_delta;

    if (current_tick == motor_stats.time_prev) {
        // If the current time is identical to the previous, we shouldn't do these calculations
        return;
    }

    // get distance travelled since we last did this
    enc_l_delta = (left_encoder_counts - motor_stats.enc_prev[kEncoder_Left]);
    enc_r_delta = (right_encoder_counts - motor_stats.enc_prev[kEncoder_Right]);

    // update position
    motor_stats.pos[kMotor_Left] += enc_l_delta;
    motor_stats.pos[kMotor_Right] += enc_r_delta;

    // update velocity... ugly
    motor_stats.vel[kMotor_Left]  = (float)enc_l_delta / (float)(current_tick - motor_stats.time_prev);
    motor_stats.vel[kMotor_Right] = (float)enc_r_delta / (float)(current_tick - motor_stats.time_prev);

    // update those parameters!
    motor_stats.enc_prev[kEncoder_Left]  = left_encoder_counts;
    motor_stats.enc_prev[kEncoder_Right] = right_encoder_counts;
    motor_stats.time_prev = current_tick;
}


/* -- Helper functions that get / set data -- */


float motor_calc_velocity_get(motor_select_t motor_desired)
{
    return motor_stats.vel[motor_desired];
}


int32_t motor_calc_distance_get(motor_select_t motor_desired)
{
    return motor_stats.pos[motor_desired];
}


void motor_calc_velocity_clear(motor_select_t motor_desired)
{
    motor_stats.vel[motor_desired] = 0;
}


void motor_calc_distance_clear(motor_select_t motor_desired)
{
    motor_stats.pos[motor_desired] = 0;
}


/* -- PID runner / helper functions -- */

void motor_calc_PID_run(float *err_output_l_ptr, float *err_output_r_ptr)
{
    float err_term[2];
    float err_output[2];

    // Run velocity PID calculations
    // first, calculate the proportional error
    if (PID_admin.mode == kPID_velocity) {
        err_term[kMotor_Left]  = PID_admin.vel_target[kMotor_Left]  - motor_stats.vel[kMotor_Left];
        err_term[kMotor_Right] = PID_admin.vel_target[kMotor_Right] - motor_stats.vel[kMotor_Right];
    } else if (PID_admin.mode == kPID_distance) {
        err_term[kMotor_Left]  = (float)(PID_admin.pos_target[kMotor_Left]  - motor_stats.pos[kMotor_Left]);
        err_term[kMotor_Right] = (float)(PID_admin.pos_target[kMotor_Right] - motor_stats.pos[kMotor_Right]);
    } else {
        // invalid use case...
        *err_output_l_ptr = 0;
        *err_output_r_ptr = 0;
        return;
    }

    // add that to the integral error
    PID_admin.err_integral[kMotor_Left]  += err_term[kMotor_Left];
    PID_admin.err_integral[kMotor_Right] += err_term[kMotor_Right];

    // apply the proportional constant
    err_output[kMotor_Left]  = err_term[kMotor_Left]  * PID_K_PROPORTIONAL;
    err_output[kMotor_Right] = err_term[kMotor_Right] * PID_K_PROPORTIONAL;

    // apply the derivative factor & constant
    err_output[kMotor_Left]  += (err_term[kMotor_Left] -
                                 PID_admin.err_prev[kMotor_Left]) * PID_K_DERIVATIVE;

    err_output[kMotor_Right] += (err_term[kMotor_Right] -
                                 PID_admin.err_prev[kMotor_Right]) * PID_K_DERIVATIVE;

    // apply the integral factor & constant
    err_output[kMotor_Left]  += PID_admin.err_integral[kMotor_Left]  * PID_K_INTEGRAL;
    err_output[kMotor_Right] += PID_admin.err_integral[kMotor_Right] * PID_K_INTEGRAL;

    // update the previous error term
    PID_admin.err_prev[kMotor_Left]  = err_term[kMotor_Left];
    PID_admin.err_prev[kMotor_Right] = err_term[kMotor_Right];

    // return the error output
    *err_output_l_ptr = err_output[kMotor_Left];
    *err_output_r_ptr = err_output[kMotor_Right];
}


void motor_calc_PID_setmode(PID_mode_t new_PID_mode)
{
    // clear the current PID parameters
    PID_admin.err_prev[kMotor_Left]  = 0;
    PID_admin.err_prev[kMotor_Right] = 0;
    PID_admin.err_integral[kMotor_Left]  = 0;
    PID_admin.err_integral[kMotor_Right] = 0;
    // set the mode
    PID_admin.mode = new_PID_mode;
}


PID_mode_t motor_calc_PID_getmode(void)
{
    return PID_admin.mode;
}


void motor_calc_PID_set_target(motor_select_t motor_desired, void * data)
{
    if ( motor_calc_PID_getmode() == kPID_velocity ) {
        PID_admin.vel_target[motor_desired] = *(float *)data;
    } else if ( motor_calc_PID_getmode() == kPID_distance ) {
        PID_admin.pos_target[motor_desired] = *(uint32_t *)data;
    }
}


bool motor_calc_PID_is_done(motor_select_t motor_desired, float err_tollerance)
{
    float err_term[2];

    if (PID_admin.mode == kPID_velocity) {
        err_term[kMotor_Left]  = PID_admin.vel_target[kMotor_Left]  - motor_stats.vel[kMotor_Left];
        err_term[kMotor_Right] = PID_admin.vel_target[kMotor_Right] - motor_stats.vel[kMotor_Right];
    } else if (PID_admin.mode == kPID_distance) {
        err_term[kMotor_Left]  = (float)(PID_admin.pos_target[kMotor_Left]  - motor_stats.pos[kMotor_Left]);
        err_term[kMotor_Right] = (float)(PID_admin.pos_target[kMotor_Right] - motor_stats.pos[kMotor_Right]);
    } else {
        // we're done I guess... :p
        return true;
    }

    if (err_term[motor_desired] <= err_tollerance) {
        return true;
    } else {
        return false;
    }
}
