/*
 * Driver functions to control the MC33926 motor controller.
 *
 * Author: Tyler Holmes (tyler@holmesengineering.com)
 * Date: 1/2/2016
 */

#ifndef _MOTOR_DRIVERS_H_
#define _MOTOR_DRIVERS_H_


/*******************************************************************************
 * Public Data Structures
 ******************************************************************************/

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


/*******************************************************************************
 * Public Function Definitions
 ******************************************************************************/

/*!
 * @brief Initializes all motor pins and sets up PWM modules. Leaves motor drivers in highZ.
 *
 * @note This function should be called before all others.
 */
void motors_init(void);

/*!
 * @brief Configures the given motor to the given mode.
 *
 * @param motor_selected (motor_select_t): Motor you want to configure.
 * @param mode (motor_mode_t): mode type you want to set the motor to.
 */
void motors_set_mode(motor_select_t motor_selected, motor_mode_t direction);

/*!
 * @brief Sets the PWM duty cycle to the given motor.
 *
 * @param motor_selected (motor_select_t): Motor you want to configure.
 * @param pwm_val (uint8_t): Duty cycle from 0 - 100.
 */
void motors_set_pwm(motor_select_t motor_selected, uint8_t pwm_val);

// Public access functions to admin struct

/*!
 * @brief Gets the PWM duty cycle the given motor has currently.
 *
 * @param motor_selected (motor_select_t): Motor you want to retrieve from.
 * @return (uint8_t): Duty cycle from 0 - 100.
 */
uint8_t motors_get_pwm(motor_select_t motor_selected);

/*!
 * @brief Gets the mode currently set for the given motor
 *
 * @param motor_selected (motor_select_t): Motor you want to retrieve from.
 * @return (motor_mode_t): Mode of the motor.
 */
motor_mode_t motors_get_mode(motor_select_t motor_selected);


/*!
 * @brief Gets the direction the motor is moving. Etiher 1, -1, or 0
 *
 * @param motor_selected (motor_select_t): Motor you want to retrieve from.
 * @return (int8_t): Direction of the motor.
 */
int8_t motors_get_direction(motor_select_t motor_selected);

#endif /* _MOTOR_DRIVERS_H_ */
