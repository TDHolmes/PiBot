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
// definitions of peripherals and memory addresses
#include "MKL03Z4.h"
#include "hardware.h"
#include "fsl_port.h"
#include "encoders.h"
// need a time module


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
    // keep track of integral error over time.
    float      vel_integral_err[2];
    int32_t    pos_integral_err[2];
} motor_PID_t;


motor_stats_t motor_stats;
motor_PID_t  PID_admin;


void motor_calc_init(void)
{
    // initialize stats struct
    motor_stats.pos[kMotor_Right] = 0;
    motor_stats.pos[kMotor_Left] = 0;
    motor_stats.vel[kMotor_Right] = 0;
    motor_stats.vel[kMotor_Left] = 0;
    motor_stats.enc_prev[kEncoder_Right] = 0;
    motor_stats.enc_prev[kEncoder_Left] = 0;
    motor_stats.time_prev = get_current_tick();

    // initialize PID struct
    PID_admin.mode = kPID_stop;
    PID_admin.vel_target[kMotor_Left] = 0;
    PID_admin.vel_target[kMotor_Right] = 0;
    PID_admin.pos_target[kMotor_Left] = 0;
    PID_admin.pos_target[kMotor_Right] = 0;
}


// function to be called periodically to update position and velocity.
void motor_calc_parameters(void)
{
    int32_t enc_l, enc_r;
    int32_t enc_l_delta, enc_r_delta;
    int32_t time;

    // get the current time and position
    time = get_current_tick();
    enc_l = encoders_get_counts(kEncoder_Left);
    enc_r = encoders_get_counts(kEncoder_Right);

    // get distance travelled since we last did this
    enc_l_delta = (enc_l - motor_stats.enc_prev[kEncoder_Left]);
    enc_r_delta = (enc_r - motor_stats.enc_prev[kEncoder_Right]);

    // update position
    motor_stats.pos[kMotor_Left] += enc_l_delta;
    motor_stats.pos[kMotor_Right] += enc_r_delta;

    // update velocity... ugly
    motor_stats.vel[kMotor_Left]  = (float)enc_l_delta / (float)(time - motor_stats.time_prev);
    motor_stats.vel[kMotor_Right] = (float)enc_r_delta / (float)(time - motor_stats.time_prev);

    // update those parameters!
    motor_stats.enc_prev[kEncoder_Left]  = enc_l;
    motor_stats.enc_prev[kEncoder_Right] = enc_r;
    motor_stats.time_prev = time;
}


/* -- Helper functions that get / set data -- */


inline float motor_calc_velocity_get(motor_select_t motor_desired)
{
    return motor_stats.vel[motor_desired];
}


inline int32_t motor_calc_distance_get(motor_select_t motor_desired)
{
    return motor_stats.pos[motor_desired];
}


inline void motor_calc_velocity_clear(motor_select_t motor_desired)
{
    motor_stats.vel[motor_desired] = 0;
}


inline void motor_calc_distance_clear(motor_select_t motor_desired)
{
    motor_stats.pos[motor_desired] = 0;
}


/* -- PID runner / helper functions -- */

void motor_calc_PID_run(void)
{
    switch (PID_admin.mode) {
        case(kPID_velocity):
            // Run velocity PID calculations
            break;

        case(kPID_distance):
            // Run distance PID calculations
            break;

        default:
            // do nothing
            break;
    }
}


inline void motor_calc_PID_setmode(PID_mode_t new_PID_mode)
{
    // clear the current PID parameters
    PID_admin.vel_integral_err[kMotor_Left]  = 0;
    PID_admin.vel_integral_err[kMotor_Right] = 0;
    PID_admin.pos_integral_err[kMotor_Left]  = 0;
    PID_admin.pos_integral_err[kMotor_Right] = 0;
    // set the mode
    PID_admin.mode = new_PID_mode;
}

