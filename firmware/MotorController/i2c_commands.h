#ifdef _I2C_COMMANDS_H_
#define _I2C_COMMANDS_H_


typedef enum {
    kIdle = 0,
    kTurnLeft = 0x10,
    kTurnRight,
    kMoveDistance,
    kSetVelocity_Left = 0x20,
    kSetVelocity_Right,
    kSetVelocity_Net,
    kClearPosition_Left = 0x30,
    kClearPosition_Right,
    kGetPosition_Left = 0x40,
    kGetPosition_Right,
    kGetVelocity_Left = 0x50,
    kGetVelocity_Right,
    kGetVelocity_Net,
} i2c_command_t;


#endif /* _I2C_COMMANDS_H_ */
