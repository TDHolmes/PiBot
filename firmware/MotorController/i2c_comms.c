/* \file i2c_comms.c
 * \brief I2C slave comms interface for the PycoBot project.
 *
 *  \Author: Tyler Holmes (tyler@holmesengineering.com)
 *  \Date: 1/2/2016
 */

// standard libraries required
#include <stdint.h>
#include <stdbool.h>
#include <string.h>  // for memcpy and memset

// definitions of peripherals and memory addresses
#include "MKL03Z4.h"
#include "constants.h"

#include "motor_calc.h"
#include "fsl_i2c.h"
#include "i2c_comms.h"

/*******************************************************************************
*                            Private Data Structures                           *
*******************************************************************************/
static uint8_t slave_data_buffer[I2C_DATA_LENGTH];
static i2c_slave_handle_t slave_handle;

static int32_t g_dist;
static float   g_vel;

// place to temporarily store a new command waiting for
static uint8_t new_command_databuffer[I2C_DATA_LENGTH];
static i2c_command_t new_command_type;

//! Enum to keep track of the I2C transaction state
typedef enum {
    kWait,             //!< Waiting for a transaction to start
    kReceivingAddr,    //!< Receiving an address from the master to write to or read from
    kReceivingData,    //!< Receiving data to be written to the given address
    kTransmittingData, //!< Transmitting data from the requested address
} i2c_state_t;


//! Our own structure to keep track of things.
typedef struct {
    volatile uint8_t *new_command_ready_ptr;  //!< Flag to set if there's a new command ready
    uint8_t slave_address;     //!< the address we'll be referred to as
    int8_t *addr_to_use_ptr;   //!< This is the address either being writen to or read out. Valid if positive
    i2c_state_t state;         //!< state machine to keep track of the transaction state
} i2c_comms_config_t;

//! Global admin struct to keep track of everything
static i2c_comms_config_t comms_admin;

/*******************************************************************************
*                        Private Function Definitions                          *
*******************************************************************************/

static void i2c_comms_callback(I2C_Type *base, i2c_slave_transfer_t *xfer, void *userData);
static void i2c_comms_store_command(i2c_command_t command, uint8_t * data);


/*******************************************************************************
*                            Function Implementations                          *
*******************************************************************************/

void i2c_comms_init(uint8_t slave_address, volatile uint8_t * new_command_ready_ptr)
{
    // configure the admin struct
    comms_admin.new_command_ready_ptr = new_command_ready_ptr;
    comms_admin.slave_address = slave_address;
    *comms_admin.addr_to_use_ptr = -127;
    comms_admin.state = kWait;

    // I2C slave configuration (can't modify the pointer, but can modify contents)
    i2c_slave_config_t i2c_conf;
    i2c_slave_config_t * i2c_conf_ptr = &i2c_conf;

    #pragma GCC diagnostic ignored "-Wuninitialized"
    // Get some sane defaults loaded
    I2C_SlaveGetDefaultConfig(i2c_conf_ptr);

    // add in I2C as a wake source and configure the address
    i2c_conf.enableWakeUp = true;
    i2c_conf.addressingMode = kI2C_Address7bit;
    i2c_conf.slaveAddress = comms_admin.slave_address;
    i2c_conf.upperAddress = 0;

    // initialize the I2C slave
    I2C_SlaveInit(I2C0, i2c_conf_ptr);

    // Clear the data!
    memset(&slave_handle, 0, sizeof(slave_handle));
    I2C_SlaveTransferCreateHandle(I2C0, &slave_handle, i2c_comms_callback, NULL);

    // start listening for the events defined in the bit mask
    I2C_SlaveTransferNonBlocking(I2C0, &slave_handle,
        kI2C_SlaveTransmitEvent | kI2C_SlaveReceiveEvent | kI2C_SlaveCompletionEvent);
}


// this function returns a pointer to the requested data
static void * get_data_for_address(int8_t address)
{
    // the address we advertise is the same as the command value
    switch ((i2c_command_t)address) {
        case kGetPosition_Left:
            g_dist = motor_calc_distance_get(kMotor_Left);
            return &g_dist;

        case kGetPosition_Right:
            g_dist = motor_calc_distance_get(kMotor_Right);
            return &g_dist;

        case kGetVelocity_Left:
            g_vel = motor_calc_velocity_get(kMotor_Left);
            return &g_vel;

        case kGetVelocity_Right:
            g_vel = motor_calc_velocity_get(kMotor_Right);
            return &g_vel;

        default:
            // invalid address for getting data. Return null pointer
            return 0;
    }
}


// Gets called to handle I2C transactions
static void i2c_comms_callback(I2C_Type *base, i2c_slave_transfer_t *xfer, void *userData)
{
    void *data_ptr;
    uint8_t tmp_var;
    switch (xfer->event)
    {
        /*  Receive request */
        case kI2C_SlaveReceiveEvent:
            // this function is called at the beginning of a slave receive event or when
            // we've received all data we expect but master is still sending data.
            if (comms_admin.state == kWait) {
                // if this is the start, we expect an address
                xfer->data = (uint8_t *)comms_admin.addr_to_use_ptr;
                xfer->dataSize = 1;
                comms_admin.state = kReceivingAddr;
                break;

            }
            if (comms_admin.state == kReceivingAddr) {
                // we've received the address, time to read in the data.
                comms_admin.state = kReceivingData;

            } else if (comms_admin.state == kReceivingData){
                // We must need to set more data! Go to the next address
                if (*comms_admin.addr_to_use_ptr < END_OF_SET_COMMANDS) {
                    *comms_admin.addr_to_use_ptr += 1;
                } else {
                    *comms_admin.addr_to_use_ptr = START_OF_SET_COMMANDS;
                }

            } else {
                // We are in a weird state... write into dummy variable I guess?
                tmp_var = 0;
                xfer->data = &tmp_var;
                xfer->dataSize = 1;
            }

            // clear the buffer
            memset(slave_data_buffer, 0, I2C_DATA_LENGTH);
            xfer->data = slave_data_buffer;  // can receive up to 4 bytes

            // size varies with command
            if (*comms_admin.addr_to_use_ptr == kClearPosition_Left ||
                *comms_admin.addr_to_use_ptr == kClearPosition_Right) {
                // just a boolean
                xfer->dataSize = 1;

            } else if (*comms_admin.addr_to_use_ptr == kTurnLeft ||
                       *comms_admin.addr_to_use_ptr == kTurnLeft) {
                // 0 -> 180 unsigned is uint8_t
                xfer->dataSize = 1;

            } else if (*comms_admin.addr_to_use_ptr == kSetVelocity_Left ||
                       *comms_admin.addr_to_use_ptr == kSetVelocity_Right) {
                // velocity is a 4 byte single percision float
                xfer->dataSize = 4;

            } else if (*comms_admin.addr_to_use_ptr == kMoveDistance_Left ||
                       *comms_admin.addr_to_use_ptr == kMoveDistance_Right) {
                // distance is a int32_t in mm
                xfer->dataSize = 4;
            }
            break;

        /*  Transmit request */
        case kI2C_SlaveTransmitEvent:
            // this function gets called either at the start of the slave transmitting data
            // or when the slave has already transmitted all information given and the master
            // is requesting more.
            if (comms_admin.state == kReceivingAddr) {
                // we've received an address, now to reply with the requested data.
                data_ptr = get_data_for_address(*comms_admin.addr_to_use_ptr);
                xfer->data = data_ptr;
                // size varies with command
                if (*comms_admin.addr_to_use_ptr == kClearPosition_Left ||
                    *comms_admin.addr_to_use_ptr == kClearPosition_Right) {
                    // just a boolean
                    xfer->dataSize = 1;

                } else if (*comms_admin.addr_to_use_ptr == kTurnLeft ||
                           *comms_admin.addr_to_use_ptr == kTurnLeft) {
                    // 0 -> 180 unsigned is uint8_t
                    xfer->dataSize = 1;

                } else if (*comms_admin.addr_to_use_ptr == kSetVelocity_Left ||
                           *comms_admin.addr_to_use_ptr == kSetVelocity_Right) {
                    // velocity is a 4 byte single percision float
                    xfer->dataSize = 4;

                } else if (*comms_admin.addr_to_use_ptr == kMoveDistance_Left ||
                           *comms_admin.addr_to_use_ptr == kMoveDistance_Right) {
                    // distance is a int32_t in mm
                    xfer->dataSize = 4;
                }
                comms_admin.state = kTransmittingData;

            } else if (comms_admin.state == kTransmittingData) {
                // If we're still getting a request for data from the master, they must
                // want the next address.
                if (*comms_admin.addr_to_use_ptr < END_OF_GET_COMMANDS) {
                    *comms_admin.addr_to_use_ptr += 1;
                } else {
                    *comms_admin.addr_to_use_ptr = START_OF_GET_COMMANDS;
                }
                data_ptr = get_data_for_address(*comms_admin.addr_to_use_ptr);
                xfer->data = (uint8_t *)data_ptr;
                xfer->dataSize = sizeof(*data_ptr);

            } else {
                // we're in a bad state... Send out 0's
                tmp_var = 0;
                xfer->data = &tmp_var;
                xfer->dataSize = 1;
            }
            break;

        /*  Transfer done */
        case kI2C_SlaveCompletionEvent:
            if (comms_admin.state == kReceivingData) {
                // write that data back to the appropriate location
                i2c_comms_store_command(*comms_admin.addr_to_use_ptr, slave_data_buffer);
            }

            // -- Do the command it requested -- //
            /* If we're just clearing variables, do that here. Otherwise,
             * We should change the state of the main loop!
             */
            if (*comms_admin.addr_to_use_ptr == kClearPosition_Left) {
                motor_calc_distance_clear(kMotor_Left);
            } else if (*comms_admin.addr_to_use_ptr == kClearPosition_Right) {
                motor_calc_distance_clear(kMotor_Right);
            } else {
                // Switch main to do whatever command we just received.
                i2c_comms_store_command(*comms_admin.addr_to_use_ptr, slave_data_buffer);
            }

            // reset the address of interest to an invalid address and reset the state machine
            *comms_admin.addr_to_use_ptr = -127;
            comms_admin.state = kWait;
            break;
    }
}


void i2c_comms_store_command(i2c_command_t command, uint8_t * data)
{
    new_command_type = command;
    // copy the data from data (the source) into new_command_databuffer
    // to be stored until the application calls `get_new_command`
    memcpy(new_command_databuffer, data, I2C_DATA_LENGTH);
    // raise the flag that a new command is ready!
    *comms_admin.new_command_ready_ptr = 1;
}


i2c_command_t i2c_comms_get_command(uint8_t * data_buffer)
{
    memcpy(data_buffer, new_command_databuffer, I2C_DATA_LENGTH);
    // reset the flag
    *comms_admin.new_command_ready_ptr = 0;
    return new_command_type;
}
