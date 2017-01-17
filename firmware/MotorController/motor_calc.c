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


//! Keeps track of salient motor stats.
typedef struct {
    float   vel_l;
    float   vel_r;
    int32_t pos_l;
    int32_t pos_r;
    int32_t enc_l_prev;
    int32_t enc_r_prev;
    int32_t time_prev;
} motor_stats_t;

motor_stats_t motor_stats;


void motor_calc_init(void)
{
    // initialize struct
    motor_stats.pos_l = 0;
    motor_stats.pos_r = 0;
    motor_stats.vel_l = 0;
    motor_stats.vel_r = 0;
    motor_stats.enc_l_prev = 0;
    motor_stats.enc_r_prev = 0;
    motor_stats.time_prev = get_current_tick();
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
    enc_l_delta = (enc_l - motor_stats.enc_l_prev);
    enc_r_delta = (enc_r - motor_stats.enc_r_prev);

    // update position
    motor_stats.pos_l += enc_l_delta;
    motor_stats.pos_r += enc_r_delta;

    // update velocity... ugly
    motor_stats.vel_l = (float)enc_l_delta / (float)(time - motor_stats.time_prev);
    motor_stats.vel_r = (float)enc_r_delta / (float)(time - motor_stats.time_prev);

    // update those parameters!
    motor_stats.enc_l_prev = enc_l;
    motor_stats.enc_r_prev = enc_r;
    motor_stats.time_prev = time;
}


inline float motor_calc_velocity_get(enc_type_t enc_desired)
{
    if (enc_desired == kEncoder_Left) {
        return motor_stats.vel_l;
    } else if (enc_desired == kEncoder_Right) {
        return motor_stats.vel_r;
    }
}


inline int32_t motor_calc_distance_get(enc_type_t enc_desired)
{
    if (enc_desired == kEncoder_Left) {
        return motor_stats.pos_l;
    } else if (enc_desired == kEncoder_Right) {
        return motor_stats.pos_r;
    }
}


inline void motor_calc_velocity_clear(enc_type_t enc_desired)
{
    if (enc_desired == kEncoder_Left) {
        motor_stats.vel_l = 0;
    } else if (enc_desired == kEncoder_Right) {
        motor_stats.vel_r = 0;
    }
}


inline void motor_calc_distance_clear(enc_type_t enc_desired)
{
    if (enc_desired == kEncoder_Left) {
        motor_stats.pos_l = 0;
    } else if (enc_desired == kEncoder_Right) {
        motor_stats.pos_r = 0;
    }
}

