#ifndef _CONSTANTS_H_
#define _CONSTANTS_H_


/* ------------------ Useful macros ------------------ */

#if !defined(MIN)
#define MIN(a, b) ((a) >= (b) ? (b) : (a))
#endif

#if !defined(MAX)
#define MAX(a, b) ((a) < (b) ? (b) : (a))
#endif


/* ------------------ Motor Constants --------------------- */

#define MOTOR_CALC_UPDATE_RATE_MS (1)
#define MM_PER_ENCODER_TICK       (4)

//! Used to select which motor driver you want to configure/retrieve.
typedef enum {
    kMotor_Left,
    kMotor_Right
} motor_select_t;


//! Direction modes possible.
typedef enum {
    kMotor_Mode_Forward,
    kMotor_Mode_Backward,
    kMotor_Mode_HighZ,
    kMotor_Mode_Brake
} motor_mode_t;


/* ------------------ Encoder Constants --------------------- */

//! Encoder type to select from.
typedef enum {
    kEncoder_Left,
    kEncoder_Right
} enc_type_t;


/* ------------------ PID Constants --------------------- */

#define PID_K_DERIVATIVE   0.2f
#define PID_K_PROPORTIONAL 3.0f
#define PID_K_INTEGRAL     1.0f

typedef enum {
	kPID_stop,
    kPID_velocity,
    kPID_distance
} PID_mode_t;

/* ----------------- Useful Constants -------------------------*/

#define PIBOT_DIAMETER_MM (100.0f)
#define PI                (3.14159265358f)


#endif /* _CONSTANTS_H_ */
