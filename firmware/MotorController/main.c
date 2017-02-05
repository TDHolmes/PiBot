/*
 * Motor Controller for the PycoBot project.
 *
 *
 *
 *
 */

#include <stdint.h>
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

    // Init board hardware.
    hw_init_pins();
    clk_conf_run();

    // initialize the motor driver and encoders
    motors_init();
    encoders_init();

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
            motor_calc_parameters();

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
                        //TODO: Need to convert from angle to linear distances for each wheel
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
                        //TODO: Need to convert from angle to linear distances for each wheel
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
                        // hacky uint32 from a 4 item uint8_t array
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
                        // hacky uint32 from a 4 item uint8_t array
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
                        // hacky uint32 from a 4 item uint8_t array
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
                        // hacky float from a 4 item uint8_t array
                        gen_purpose_float = i2c_command_data.float_val;
                        motor_calc_PID_setmode(kPID_velocity);
                        motor_calc_PID_set_target(kMotor_Left, kPID_velocity, &gen_purpose_float);
                        new_command = false;
                    }
                    break;

                case (kSetVelocity_Right):
                    if (new_command) {
                        // hacky float from a 4 item uint8_t array
                        gen_purpose_float = i2c_command_data.float_val;
                        motor_calc_PID_setmode(kPID_velocity);
                        motor_calc_PID_set_target(kMotor_Right, kPID_velocity, &gen_purpose_float);
                        new_command = false;
                    }
                    break;


                case (kSetVelocity_Both):
                    if (new_command) {
                        // hacky float from a 4 item uint8_t array
                        gen_purpose_float = i2c_command_data.float_val;
                        motor_calc_PID_setmode(kPID_velocity);
                        motor_calc_PID_set_target(kMotor_Left, kPID_velocity, &gen_purpose_float);
                        motor_calc_PID_set_target(kMotor_Right, kPID_velocity, &gen_purpose_float);
                        new_command = false;
                    }
                    break;

                default: {
                    // odd that we got here...
                }

            // run the PID loop with the updated parameters
            motor_calc_PID_run();

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
    }
    new_command = true;
}
