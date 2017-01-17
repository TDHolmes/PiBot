/*
 * Motor Controller for the PycoBot project.
 *
 *
 *
 *
 */

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"

// definitions of peripherals and memory addresses
#include "MKL03Z4.h"
#include "hardware.h"

// important user code
#include "clock_config.h"
#include "encoders.h"
#include "motor_drivers.h"

// flag to trigger superloop
extern uint8_t timer_update_parameters;

i2c_command_t active_command;
void * command_data_ptr;

/*!
 * @brief Main function
 */
int main(void)
{

    /* Init board hardware. */
    hw_init_pins();
    clk_conf_run();
    hw_init_debug_console();

    // initialize the motor driver and encoders
    motors_init();
    encoders_init();


    // Setup a timer to trigger the running of the PID loop

    PRINTF("hello world.\r\n");

    while (1)
    {
        // wait x ms between updating motor parameters
        if (timer_update_parameters) {
            run_loop = 0;
            motor_calc_parameters();
        }

        // run whatever command is currently active
        switch (active_command) {
            case (kIdle):

                break;

            case (kTurnLeft):

                if (done) {
                    active_command = kIdle;
                }
                break;

            case (kTurnRight):

                if (done) {
                    active_command = kIdle;
                }
                break;

            case (kMoveDistance):

                if (done) {
                    active_command = kIdle;
                }
                break;

            case (kSetVelocity_Left):

                if (done) {
                    active_command = kIdle;
                }
                break;

            case (kSetVelocity_Right):

                if (done) {
                    active_command = kIdle;
                }
                break;

            case (kSetVelocity_Net):

                if (done) {
                    active_command = kIdle;
                }
                break;

            case (kClearPosition_Left):

                // simple 1 cycle command. We're done
                active_command = kIdle;
                break;

            case (kClearPosition_Right):

                // simple 1 cycle command. We're done
                active_command = kIdle;
                break;

            case (kGetPosition_Left):

                // simple 1 cycle command. We're done
                active_command = kIdle;
                break;

            case (kGetPosition_Right):

                // simple 1 cycle command. We're done
                active_command = kIdle;
                break;

            case (kGetVelocity_Left):

                // simple 1 cycle command. We're done
                active_command = kIdle;
                break;

            case (kGetVelocity_Right):

                // simple 1 cycle command. We're done
                active_command = kIdle;
                break;

            case (kGetVelocity_Net):

                // simple 1 cycle command. We're done
                active_command = kIdle;
                break;

            default: {
                // odd that we got here...
            }
        }
    }
}


void set_active_command(i2c_command_t command, void * data)
{
    active_command = command;
    command_data_ptr = data_ptr;
}
