#ifndef _I2C_COMMS_H_
#define _I2C_COMMS_H_

#include <stdint.h>

/*******************************************************************************
 * Public Data Structures
 ******************************************************************************/

typedef enum {
    kIdle = 0,
    kTurnLeft,
    kTurnRight,
    kMoveDistance_Left,
    kMoveDistance_Right,
    kMoveDistance_Both,
    kSetVelocity_Left,
    kSetVelocity_Right,
    kSetVelocity_Both,
    kClearPosition_Left,
    kClearPosition_Right,
    kGetPosition_Left,
    kGetPosition_Right,
    kGetVelocity_Left,
    kGetVelocity_Right,
} i2c_command_t;

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define START_OF_SET_COMMANDS (kTurnLeft)
#define END_OF_SET_COMMANDS   (kClearPosition_Right)

#define START_OF_GET_COMMANDS (kGetPosition_Left)
#define END_OF_GET_COMMANDS   (kGetVelocity_Right)

#define I2C_ADDRESS_BASE 0x40
#define I2C_LSB_PIN     13

// when receiving, we expect a command byte and two data bytes
// when transmitting, we reserve 2 bytes for data
#define I2C_DATA_LENGTH 4

/*******************************************************************************
 * Public Function Definitions
 ******************************************************************************/

void          i2c_comms_init(uint8_t slave_address, volatile uint8_t * new_command_ready_ptr);
i2c_command_t i2c_comms_get_command(uint8_t * data_buffer);


#endif /* _I2C_COMMS_H_ */
