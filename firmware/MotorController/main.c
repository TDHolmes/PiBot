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
#include "motor_drivers.h"
#include "i2c_comms.h"


// flag to trigger superloop
volatile uint8_t timer_update_parameters;

i2c_command_t active_command;
uint8_t i2c_command_data[I2C_DATA_LENGTH];
bool new_command;


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
    hw_init_debug_console();

    // initialize the motor driver and encoders
    motors_init();
    encoders_init();

    // read B13 to twiddle the I2C LSB and initialize the I2C slave
    slave_address = I2C_ADDRESS_BASE | GPIO_ReadPinInput(GPIOB, I2C_LSB_PIN);
    i2c_comms_init(slave_address);

    // Setup a timer to trigger the running of the PID loop every N ms
    timer_init(&timer_update_parameters, MOTOR_CALC_UPDATE_RATE_MS);

    // get an initial command of the Idle state
    memset(i2c_command_data, 0, I2C_DATA_LENGTH);
    set_active_command(kIdle, i2c_command_data);

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
                        gen_purpose_uint8 = i2c_command_data[0];
                        new_command = false;
                    }

                    if (done) {
                        set_active_command(kIdle, NULL);
                    }
                    break;

                case (kTurnRight):
                    if (new_command) {
                        // lets get the angle we want to turn
                        gen_purpose_uint8 = i2c_command_data[0];
                        new_command = false;
                    }

                    if (done) {
                        set_active_command(kIdle, NULL);
                    }
                    break;

                case (kMoveDistance):
                    if (new_command) {
                        // hacky uint32 from a 4 item uint8_t array
                        gen_purpose_uint32 = *i2c_command_data;
                        new_command = false;
                    }

                    if (done) {
                        set_active_command(kIdle, NULL);
                    }
                    break;

                case (kSetVelocity_Left):
                    if (new_command) {
                        // hacky float from a 4 item uint8_t array
                        gen_purpose_float = *i2c_command_data;
                        new_command = false;
                    }

                    if (done) {
                        set_active_command(kIdle, NULL);
                    }
                    break;

                case (kSetVelocity_Right):
                    if (new_command) {
                        // hacky float from a 4 item uint8_t array
                        gen_purpose_float = *i2c_command_data;
                        new_command = false;
                    }

                    if (done) {
                        set_active_command(kIdle, NULL);
                    }
                    break;

                case (kSetVelocity_Net):
                    if (new_command) {
                        // hacky float from a 4 item uint8_t array
                        gen_purpose_float = *i2c_command_data;
                        new_command = false;
                    }

                    if (done) {
                        set_active_command(kIdle, NULL);
                    }
                    break;

                default: {
                    // odd that we got here...
                }

            // run the PID loop with the updated parameters
            motor_calc_run_PID();

            }  /* END RUN EVERY N MS */
        }
    }
}


void set_active_command(i2c_command_t command, void * data)
{
    // change the current command
    active_command = command;
    // copy the data over to our local buffer
    memcpy(i2c_command_data, data, I2C_DATA_LENGTH);
    new_command = true;
}
