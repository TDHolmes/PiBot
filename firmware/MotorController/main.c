/*
 * Motor Controller for the PycoBot project.
 *
 *
 *
 *
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>  // for memset
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"

// definitions of peripherals and memory addresses
#include "MKL03Z4.h"
#include "hardware.h"
#include "fsl_gpio.h"

// important user code
#include "clock_config.h"
#include "encoders.h"
#include "motor_calc.h"
#include "motor_drivers.h"
#include "i2c_comms.h"
#include "timer.h"
#include "main.h"


// flag to trigger superloop
volatile uint8_t timer_update_parameters;

i2c_command_t active_command;
bool new_command;


typedef union {
    uint8_t  uint8_val;
    uint32_t uint32_val;
    float    float_val;
} i2c_command_data_t;

i2c_command_data_t i2c_command_data;


int main(void)
{
    uint8_t slave_address;
    // some general purpose variables to be used in the main loop
    float    gen_purpose_float;
    uint8_t  gen_purpose_uint8;
    uint32_t gen_purpose_uint32;

    // place to store the error output when running PID calculations
    float err_output[2] = {0, 0};

    // Init board hardware.
    hw_init_pins();
    clk_conf_run();

    // initialize the motor driver and encoders
    motors_init();
    encoders_init();
    motor_calc_init( timer_get_tick() );

    // read B13 to twiddle the I2C LSB and initialize the I2C slave
    slave_address = I2C_ADDRESS_BASE | GPIO_ReadPinInput(GPIOB, I2C_LSB_PIN);
    i2c_comms_init(slave_address);

    // Setup a timer to trigger the running of the PID loop every N ms
    timer_init(&timer_update_parameters, MOTOR_CALC_UPDATE_RATE_MS);

    // get an initial command of the Idle state
    i2c_command_data.uint8_val = 0;
    i2c_command_data.uint32_val = 0;
    i2c_command_data.float_val = 0;
    set_active_command(kIdle, NULL);

    while (1)
    {
        /* wait x ms between updating motor parameters and running the active command
         * switch statement.
         */
        if (timer_update_parameters) {
            timer_update_parameters = 0;
            motor_calc_parameters( encoders_get_counts(kEncoder_Left),
                                   encoders_get_counts(kEncoder_Right),
                                   timer_get_tick() );

            // run whatever command is currently active
            switch (active_command) {

                case (kIdle):
                    if (new_command) {
                        motors_set_mode(kMotor_Left, kMotor_Mode_HighZ);
                        motors_set_mode(kMotor_Right, kMotor_Mode_HighZ);
                        new_command = false;
                    }
                    // nothing to do after we've turned off the motors
                    break;

                case (kTurnLeft):
                    if (new_command) {
                        // lets get the angle we want to turn
                        gen_purpose_uint8 = i2c_command_data.uint8_val;
                        motor_calc_PID_setmode(kPID_distance);

                        // Convert angle to distances for both motors
                        gen_purpose_uint32 = (uint32_t)(((float)gen_purpose_uint8 / 360.0f) * PIBOT_DIAMETER_MM * PI);
                        motor_calc_PID_set_target(kMotor_Right, kPID_distance, &gen_purpose_uint32);
                        gen_purpose_uint32 = (-1 * gen_purpose_uint32);
                        motor_calc_PID_set_target(kMotor_Left, kPID_distance, &gen_purpose_uint32);
                        new_command = false;
                    }

                    if (motor_calc_PID_is_done(kMotor_Left, 0.1) && motor_calc_PID_is_done(kMotor_Right, 0.1)) {
                        set_active_command(kIdle, NULL);
                        motor_calc_PID_setmode(kPID_stop);
                    }
                    break;

                case (kTurnRight):
                    if (new_command) {
                        // lets get the angle we want to turn
                        gen_purpose_uint8 = i2c_command_data.uint8_val;
                        motor_calc_PID_setmode(kPID_distance);

                        // Convert angle to distances for both motors
                        gen_purpose_uint32 = (uint32_t)(((float)gen_purpose_uint8 / 360.0f) * PIBOT_DIAMETER_MM * PI);
                        motor_calc_PID_set_target(kMotor_Left, kPID_distance, &gen_purpose_uint32);
                        gen_purpose_uint32 = (-1 * gen_purpose_uint32);
                        motor_calc_PID_set_target(kMotor_Right, kPID_distance, &gen_purpose_uint32);
                        new_command = false;
                    }

                    if (motor_calc_PID_is_done(kMotor_Left, 0.1) && motor_calc_PID_is_done(kMotor_Right, 0.1)) {
                        set_active_command(kIdle, NULL);
                        motor_calc_PID_setmode(kPID_stop);
                    }
                    break;

                case (kMoveDistance_Left):
                    if (new_command) {
                        // We expect the uint32 part of the enum to have been set before hand
                        gen_purpose_uint32 = i2c_command_data.uint32_val;
                        motor_calc_PID_setmode(kPID_distance);
                        motor_calc_PID_set_target(kMotor_Left, kPID_distance, &gen_purpose_uint32);
                        new_command = false;
                    }

                    if (motor_calc_PID_is_done(kMotor_Left, 0.1)) {
                        set_active_command(kIdle, NULL);
                        motor_calc_PID_setmode(kPID_stop);
                    }
                    break;


                case (kMoveDistance_Right):
                    if (new_command) {
                        // We expect the uint32 part of the enum to have been set before hand
                        gen_purpose_uint32 = i2c_command_data.uint32_val;
                        motor_calc_PID_setmode(kPID_distance);
                        motor_calc_PID_set_target(kMotor_Right, kPID_distance, &gen_purpose_uint32);
                        new_command = false;
                    }

                    if (motor_calc_PID_is_done(kMotor_Right, 0.1)) {
                        set_active_command(kIdle, NULL);
                        motor_calc_PID_setmode(kPID_stop);
                    }
                    break;

                case (kMoveDistance_Both):
                    if (new_command) {
                        // We expect the uint32 part of the enum to have been set before hand
                        gen_purpose_uint32 = i2c_command_data.uint32_val;
                        motor_calc_PID_setmode(kPID_distance);
                        motor_calc_PID_set_target(kMotor_Right, kPID_distance, &gen_purpose_uint32);
                        motor_calc_PID_set_target(kMotor_Left, kPID_distance, &gen_purpose_uint32);
                        new_command = false;
                    }

                    if (motor_calc_PID_is_done(kMotor_Left, 0.1) && motor_calc_PID_is_done(kMotor_Right, 0.1)) {
                        set_active_command(kIdle, NULL);
                        motor_calc_PID_setmode(kPID_stop);
                    }
                    break;

                case (kSetVelocity_Left):
                    if (new_command) {
                        // We expect the float value to have been set
                        gen_purpose_float = i2c_command_data.float_val;
                        motor_calc_PID_setmode(kPID_velocity);
                        motor_calc_PID_set_target(kMotor_Left, kPID_velocity, &gen_purpose_float);
                        new_command = false;
                    }
                    // we never leave this state until we get told to switch states
                    break;

                case (kSetVelocity_Right):
                    if (new_command) {
                        // We expect the float value to have been set
                        gen_purpose_float = i2c_command_data.float_val;
                        motor_calc_PID_setmode(kPID_velocity);
                        motor_calc_PID_set_target(kMotor_Right, kPID_velocity, &gen_purpose_float);
                        new_command = false;
                    }
                    // we never leave this state until we get told to switch states
                    break;


                case (kSetVelocity_Both):
                    if (new_command) {
                        // We expect the float value to have been set
                        gen_purpose_float = i2c_command_data.float_val;
                        motor_calc_PID_setmode(kPID_velocity);
                        motor_calc_PID_set_target(kMotor_Left, kPID_velocity, &gen_purpose_float);
                        motor_calc_PID_set_target(kMotor_Right, kPID_velocity, &gen_purpose_float);
                        new_command = false;
                        // we never leave this state until we get told to switch states
                    }
                    break;

                default: {
                    // odd that we got here...
                }

                // run the PID loop with the updated parameters
                if (motor_calc_PID_getmode() != kPID_stop) {
                    motor_calc_PID_run( &err_output[kMotor_Left], &err_output[kMotor_Right] );

                    // compensate for the direciton the motor is going
                    err_output[kMotor_Left]  *= motors_get_direction(kMotor_Left);
                    err_output[kMotor_Right] *= motors_get_direction(kMotor_Right);

                    // saturate to 0 - 100 as that is the PWM range.
                    // TODO: Need to tune PID values to target 0 - 100 values
                    err_output[kMotor_Left]  = MAX(0, MIN(100, err_output[kMotor_Left]));
                    err_output[kMotor_Right] = MAX(0, MIN(100, err_output[kMotor_Right]));

                    // update the motors! Direction should have been taken care of elsewhere
                    motors_set_pwm(kMotor_Left,  (uint8_t)err_output[kMotor_Left]);
                    motors_set_pwm(kMotor_Right, (uint8_t)err_output[kMotor_Right]);
                }

            }  /* END RUN EVERY N MS */
        }
    }
}


void set_active_command(i2c_command_t command, void * data)
{
    // change the current command
    active_command = command;
    // copy the data over to our local buffer

    if ((command == kSetVelocity_Left) || (command == kSetVelocity_Right) || (command == kSetVelocity_Both)) {
        i2c_command_data.float_val = *(float *)data;

    } else if ((command == kMoveDistance_Left) || (command == kMoveDistance_Right) || (command == kMoveDistance_Both)) {
        i2c_command_data.uint32_val = *(uint32_t *)data;

    } else if ((command == kTurnLeft) || (command == kTurnRight)) {
        i2c_command_data.uint8_val = *(uint8_t *)data;

    } else {
        i2c_command_data.float_val = 0;
        i2c_command_data.uint32_val = 0;
        i2c_command_data.uint8_val = 0;
    }
    new_command = true;
}
