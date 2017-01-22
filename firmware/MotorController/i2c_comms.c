/*
 * I2C comms interface for the PycoBot project.
 *
 *
 *
 *
 */

// standard libraries required
#include <stdint.h>
#include <string.h>  // for memcopy and memset

// definitions of peripherals and memory addresses
#include "MKL03Z4.h"
#include "hardware.h"
#include "fsl_i2c.h"


uint8_t slave_data_buffer[I2C_DATA_LENGTH];
i2c_slave_handle_t slave_handle;


//! Enum to keep track of the I2C transaction state
typedef enum {
    kWait,
    kReceivingAddr,
    kReceivingData,
    kTransmittingData,
} i2c_state_t;


//! Our own structure to keep track of things.
typedef struct {
    uint8_t slave_address;          //! the address we'll be referred to as
    int8_t *addr_to_use_ptr;        //! This is the address either being writen to or read out. Valid if positive
    i2c_state_t comms_admin.state;  //! state machine to keep track of the transaction state
} i2c_comms_config_t;


i2c_comms_config_t comms_admin;


void i2c_comms_init(uint8_t slave_address)
{
    // configure the admin struct
    comms_admin.address = slave_address;
    *comms_admin.addr_to_use_ptr = -127;
    comms_admin.state = kWait;

    // I2C slave configuration (can't modify the pointer, but can modify contents)
    i2c_slave_config_t * const i2c_conf_ptr;
    // Get some sane defaults loaded
    I2C_SlaveGetDefaultConfig(i2c_conf_ptr);

    // add in I2C as a wake source and configure the address
    i2c_conf_ptr->enableWakeUp = true;
    i2c_conf_ptr->addressingMode = kI2C_Address7bit;
    i2c_conf_ptr->slaveAddress = comms_admin.address;
    i2c_conf_ptr->upperAddress = 0; /*  not used for this example */


    // initialize the I2C slave
    I2C_SlaveInit(I2C0, i2c_conf_ptr);

    memset(&slave_handle, 0, sizeof(slave_handle));
    I2C_SlaveTransferCreateHandle(comms_admin.slave_address, &slave_handle, i2c_comms_callback, NULL);

    // start listening for the events defined in the bit mask
    I2C_SlaveTransferNonBlocking(comms_admin.slave_address, &slave_handle,
        kI2C_SlaveTransmitEvent | kI2C_SlaveReceiveEvent | kI2C_SlaveCompletionEvent);
}


// this function returns a pointer to the requested a
static void * get_data_for_address(int8_t address)
{
    i2c_command_t command = address;

    switch ((i2c_command_t)command) {
        case kGetPosition_Left:
            // do stuff
            int32_t dist = motor_calc_distance_get(kEncoder_Left);
            return &dist;

        case kGetPosition_Right:
            // do stuff
            int32_t dist = motor_calc_distance_get(kEncoder_Right);
            return &dist;

        case kGetVelocity_Left:
            // do stuff
            float vel = motor_calc_velocity_get(kEncoder_Left);
            return &vel;

        case kGetVelocity_Right:
            // do stuff
            float vel = motor_calc_velocity_get(kEncoder_Right);
            return &vel;

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
        /*  Transmit request */
        case kI2C_SlaveTransmitEvent:
            // this function gets called either at the start of the slave transmitting data
            // or when the slave has already transmitted all information given and the master
            // is requesting more.
            if (comms_admin.state == kReceivingAddr) {
                // we've received an address, now to reply with the requested data.
                data_ptr = get_data_for_address(*comms_admin.addr_to_use_ptr)
                xfer->data = (uint8_t *)data_ptr;
                xfer->dataSize = sizeof(*data_ptr);
                comms_admin.state = kTransmittingData;

            } else if (comms_admin.state == kTransmittingData) {
                // If we're still getting a request for data from the master, they must
                // want the next address.
                *comms_admin.addr_to_use_ptr += 1;
                data_ptr = get_data_for_address(*comms_admin.addr_to_use_ptr)
                xfer->data = (uint8_t *)data_ptr;
                xfer->dataSize = sizeof(*data_ptr);

            } else {
                // we're in a bad state... Send out 0's
                tmp_var = 0;
                xfer->data = &tmp_var;
                xfer->dataSize = 1;
            }
            break;

        /*  Receive request */
        case kI2C_SlaveReceiveEvent:
            // this function is called at the beginning of a slave receive event or when
            // we've received all data we expect but master is still sending data.
            if (comms_admin.state == kWait) {
                // if this is the start, we expect an address
                xfer->data = comms_admin.addr_to_use_ptr;
                xfer->dataSize = 1;
                comms_admin.state = kReceivingAddr;
                break;

            }
            if (comms_admin.state == kReceivingAddr) {
                // we've received the address, time to read in the data.
                comms_admin.state = kReceivingData;

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
               p321 xfer->dataSize = 1;

            } else if (*comms_admin.addr_to_use_ptr == kSetVelocity_Left ||
                       *comms_admin.addr_to_use_ptr == kSetVelocity_Right) {
                // velocity is a 4 byte single percision float
                xfer->dataSize = 4;

            } else if (*comms_admin.addr_to_use_ptr == kMoveDistance_Left ||
                       *comms_admin.addr_to_use_ptr == kMoveDistance_Right) {
                // distance is a uint32_t in mm
                xfer->dataSize = 4;
            }
            break;

        /*  Transfer done */
        case kI2C_SlaveCompletionEvent:
            if (comms_admin.state == kReceivingData) {
                // write that data back to the appropriate location
                set_active_command(*comms_admin.addr_to_use_ptr, slave_data_buffer);
            }

            // -- Do the command it requested -- //
            /* If we're just clearing variables, do that here. Otherwise,
             * We should change the state of the main loop!
             */
            if (*comms_admin.addr_to_use_ptr == kClearPosition_Left) {
                motor_calc_distance_clear(kEncoder_Left);
            } else if (*comms_admin.addr_to_use_ptr == kClearPosition_Right) {
                motor_calc_distance_clear(kEncoder_Right);
            } else {
                // Switch main to do whatever command we just received.
                set_active_command(*comms_admin.addr_to_use_ptr, slave_data_buffer);
            }

            // reset the address of interest to an invalid address and reset the state machine
            *comms_admin.addr_to_use_ptr = -127;
            comms_admin.state = kWait;
            break;
    }
}
