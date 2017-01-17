#ifdef _I2C_COMMS_H_
#define _I2C_COMMS_H_


typedef enum {
    kIdle = 0,
    kTurnLeft,
    kTurnRight,
    kMoveDistance_Left,
    kMoveDistance_Right,
    kSetVelocity_Left,
    kSetVelocity_Right,
    kClearPosition_Left,
    kClearPosition_Right,
    kGetPosition_Left,
    kGetPosition_Right,
    kGetVelocity_Left,
    kGetVelocity_Right,
} i2c_command_t;

#define START_OF_SET_COMMANDS (kTurnLeft)
#define END_OF_SET_COMMANDS   (kClearPosition_Right)

#define START_OF_GET_COMMANDS (kGetPosition_Left)
#define END_OF_GET_COMMANDS   (kGetVelocity_Right)


#endif /* _I2C_COMMS_H_ */
