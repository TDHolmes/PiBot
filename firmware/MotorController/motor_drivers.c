/*
 * Driver functions to control the MC33926 motor controller.
 *
 * Author: Tyler Holmes (tyler@holmesengineering.com)
 * Date: 1/2/2016
 */

#include <stdint.h>
// definitions of peripherals and memory addresses
#include "MKL03Z4.h"
#include "hardware.h"
#include "fsl_common.h"
#include "fsl_port.h"
#include "fsl_tpm.h"  /* PWM driver */
#include "motor_drivers.h"

/* Analog feedback and error flags */
#define MOTOR_R_FB_PORT GPIOB
#define MOTOR_R_FB_PIN  0
#define MOTOR_LEFT_FB_PORT GPIOA
#define MOTOR_L_FB_PIN  8
#define MOTOR_ERR_FLAG_PORT_n GPIOA
#define MOTOR_ERR_FLAG_PIN_n  9

/* PWM / HIGH-Z output */
#define MOTOR_R_DISABLE_PORT_n GPIOA
#define MOTOR_R_DISABLE_PIN_n 12
#define MOTOR_L_DISABLE_PORT_n GPIOB
#define MOTOR_L_DISABLE_PIN_n 10

/* Direction control. Left/right get same input, with
 the left motor having an invert option. */
#define MOTOR_IN_1_PORT GPIOA
#define MOTOR_IN_1_PIN  3
#define MOTOR_IN_2_PORT GPIOA
#define MOTOR_IN_2_PIN  4
#define MOTOR_L_INVERT_PORT_n GPIOB
#define MOTOR_L_INVERT_PIN_n 11

#define MOTOR_L_PWM_CHANNEL TPM0
#define MOTOR_R_PWM_CHANNEL TPM1


//! Administration structure to keep track of the PWM and direction of each motor.
typedef struct {
    uint8_t pwm_vals[2];
    motor_mode_t modes[2];
} motor_admin_t;

motor_admin_t md_admin;


void motors_init(void)
{
    // status variable to ensure the PWM module inits fine
    status_t status;

    // config to be used for both PWM pins
    tpm_config_t pwm_config;
    // set sane defaults
    TPM_GetDefaultConfig(&pwm_config);

    // Configure specific signal parameters for the PWM output. Start with 100% duty cycle.
    tpm_chnl_pwm_signal_param_t pwm_signal_parameters;
    pwm_signal_parameters.chnlNumber = kTPM_Chnl_0;  /* ch 0-7. SOC specific. */
    pwm_signal_parameters.level = kTPM_HighTrue;     /* Logic level */
    pwm_signal_parameters.dutyCyclePercent = 0;      /* percentage for the Duty cycle. 0 - 100 expected. */

    /* Do the basic init off the TPM modules */
    TPM_Init(MOTOR_L_PWM_CHANNEL, &pwm_config);
    TPM_Init(MOTOR_R_PWM_CHANNEL, &pwm_config);

    /* Initialize the PWM modules. 100% duty cycle, 1 channel, edge alligned, 20 kHz PWM
    frequency with 48 MHz system clock as source. */

    // channel 0
    status = TPM_SetupPwm(MOTOR_L_PWM_CHANNEL, &pwm_signal_parameters, 1,
                          kTPM_EdgeAlignedPwm, 20000U, 48000000U);
    if (status != kStatus_Success) {
        // uh oh!
        assert(false);
    }
    // channel 1
    status = TPM_SetupPwm(MOTOR_R_PWM_CHANNEL, &pwm_signal_parameters, 1,
                          kTPM_EdgeAlignedPwm, 20000U, 48000000U);
    if (status != kStatus_Success) {
        // uh oh!
        assert(false);
    }

    // not sure this is necessary
    TPM_StartTimer(MOTOR_L_PWM_CHANNEL, kTPM_SystemClock);
    TPM_StartTimer(MOTOR_R_PWM_CHANNEL, kTPM_SystemClock);

    // initially turn motors off
    motors_set_mode(kMotor_Left, kMotor_Mode_HighZ);
    motors_set_mode(kMotor_Right, kMotor_Mode_HighZ);
}


void motors_set_mode(motor_select_t motor_selected, motor_mode_t mode)
{
    // check if we need to even update...
    if (md_admin.modes[motor_selected] == mode) {
        return;
    }

    // first, set motor to highZ to avoid nastiness
    motors_set_pwm(motor_selected, 0);

    // now, if we need to do something other than highZ, do that.
    if (mode == kMotor_Mode_Forward || mode == kMotor_Mode_Backward) {
        // Depending on which motor we're using, this gets interesting
        if (motor_selected == kMotor_Left) {
            // the left motor only needs to flip the left inv line
            GPIO_TogglePinsOutput(MOTOR_L_INVERT_PORT_n, (1 << MOTOR_L_INVERT_PIN_n));

        } else if (motor_selected == kMotor_Right) {
            // the right motor requires we flip both the mode pins and left invert line
            GPIO_TogglePinsOutput(MOTOR_IN_1_PORT, (1 << MOTOR_IN_1_PIN));
            GPIO_TogglePinsOutput(MOTOR_IN_2_PORT, (1 << MOTOR_IN_2_PIN));
            GPIO_TogglePinsOutput(MOTOR_L_INVERT_PORT_n, (1 << MOTOR_L_INVERT_PIN_n));
        }
        // re-enable the PWM values
        motors_set_pwm(kMotor_Left, md_admin.pwm_vals[motor_selected]);
        motors_set_pwm(kMotor_Right, md_admin.pwm_vals[motor_selected]);

    } else if (mode == kMotor_Mode_Brake) {
        // Limitation: Must brake on both motors
        // Set inputs to LOW and turn MOTOR_L_INV_n to HIGH
        GPIO_ClearPinsOutput(MOTOR_IN_1_PORT, (1 << MOTOR_IN_1_PIN));
        GPIO_ClearPinsOutput(MOTOR_IN_2_PORT, (1 << MOTOR_IN_2_PIN));
        GPIO_SetPinsOutput(MOTOR_L_INVERT_PORT_n, (1 << MOTOR_L_INVERT_PIN_n));

        // Don't PWM the motor driver when braking
        motors_set_pwm(kMotor_Left, 100);
        motors_set_pwm(kMotor_Right, 100);

    } else if (mode == kMotor_Mode_HighZ) {
        // do nothing as of now
    } else {
        assert(false);
    }
    // update the admin structure
    md_admin.modes[motor_selected] = mode;
}


void motors_set_pwm(motor_select_t motor_selected, uint8_t pwm_val)
{
    // Ensure the PWM value is 0 to 100
    pwm_val = MIN(pwm_val, 100);

    // select which motor we're talking about
    if (motor_selected == kMotor_Left) {
        TPM_UpdatePwmDutycycle(MOTOR_L_PWM_CHANNEL, kTPM_Chnl_0,
                               kTPM_EdgeAlignedPwm, pwm_val);
    } else if (motor_selected == kMotor_Right) {
        TPM_UpdatePwmDutycycle(MOTOR_R_PWM_CHANNEL, kTPM_Chnl_0,
                               kTPM_EdgeAlignedPwm, pwm_val);
    } else {
        // not good...
        assert(false);
    }
    // update the admin structure value
    md_admin.pwm_vals[motor_selected] = pwm_val;
}

// Public access functions to admin struct
uint8_t motors_get_pwm(motor_select_t motor_selected)
{
    return md_admin.pwm_vals[motor_selected];
}


motor_mode_t motors_get_mode(motor_select_t motor_selected)
{
    return md_admin.modes[motor_selected];
}

int8_t motors_get_direction(motor_select_t motor_selected)
{
    if (md_admin.modes[motor_selected] == kMotor_Mode_Forward) {
        return 1;
    } else if (md_admin.modes[motor_selected] == kMotor_Mode_Backward) {
        return -1;
    } else {
        return 0;
    }
}


bool motor_in_fault(void)
{
    if (GPIO_ReadPinInput(MOTOR_ERR_FLAG_PORT_n, MOTOR_ERR_FLAG_PIN_n)) {
        return false;
    } else {
        return true;
    }
}

