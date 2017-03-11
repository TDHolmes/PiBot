/*
 * Encoder handler functions and definitions.
 *
 *
 * Author: Tyler Holmes (tyler@holmesengineering.com)
 * Date: 1/2/2016
 */

// libraries
#include <stdbool.h>
#include <stdint.h>
// definitions of peripherals and memory addresses
#include "MKL03Z4.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "encoders.h"

#define PORTB_IRQ           PORTB_IRQn
#define ENCODER_IRQ_HANDLER PORTB_IRQHandler


// Encoder masks to determine which pins triggered an interrupt
#define ENC_RIGHT_A_MASK 0b00000010U
#define ENC_RIGHT_B_MASK 0b00000100U
#define ENC_LEFT_A_MASK  0b01000000U
#define ENC_LEFT_B_MASK  0b10000000U

// Encoder pin numbers
#define ENC_RIGHT_A 0x01U
#define ENC_RIGHT_B 0x02U
#define ENC_LEFT_A  0x06U
#define ENC_LEFT_B  0x07U


/* Private Data Structures */
// should these counts be volatile because they're updated
// in interrupt context?
typedef struct {
    int32_t left_count;
    int32_t right_count;
} encoders_t;

static encoders_t enc_admin;
static encoders_t* enc_admin_ptr;

/* Private Function Declarations */


/* Function Definitions */

void encoders_init(void)
{
    // configure admin pointer
    enc_admin_ptr = &enc_admin;
    // clear counts
    enc_admin_ptr->left_count = 0;
    enc_admin_ptr->right_count = 0;

    // Set up the encoder pins to interrupt on both edges
    // Right motor encoders
    PORT_SetPinInterruptConfig(PORTB, ENC_RIGHT_A, kPORT_InterruptEitherEdge);
    PORT_SetPinInterruptConfig(PORTB, ENC_RIGHT_B, kPORT_InterruptEitherEdge);
    // Left motor encoders
    PORT_SetPinInterruptConfig(PORTB, ENC_LEFT_A, kPORT_InterruptEitherEdge);
    PORT_SetPinInterruptConfig(PORTB, ENC_LEFT_B, kPORT_InterruptEitherEdge);
    /*Other interrupt options:
        kPORT_InterruptLogicZero
        kPORT_InterruptRisingEdge
        kPORT_InterruptFallingEdge
        kPORT_InterruptEitherEdge
        kPORT_InterruptLogicOne
    */

    // enable the interrupts
    EnableIRQ(PORTB_IRQ);
}


int32_t encoders_get_counts(enc_type_t encoder_type)
{
    if (encoder_type == kEncoder_Left) {
        return enc_admin.left_count;
    } else if (encoder_type == kEncoder_Right) {
        return enc_admin.right_count;
    } else {
        assert(false);
        return 0;
    }
}


void encoders_reset_counts(enc_type_t encoder_type)
{
    if (encoder_type == kEncoder_Left) {
        enc_admin.left_count = 0;
    } else if (encoder_type == kEncoder_Right) {
        enc_admin.right_count = 0;
    }
}


void ENCODER_IRQ_HANDLER(void)
{
    // get the mask of pins with interrupt flags
    uint32_t pin_mask = GPIO_GetPinsInterruptFlags(GPIOB);

    // check pin states. Assumes only one pin has trigerred.
    /* --- RIGHT MOTOR CHECK --- */
    if (pin_mask & ENC_RIGHT_A_MASK) {
        // encoder A has changed. High or low?
        if (GPIO_ReadPinInput(GPIOB, ENC_RIGHT_A)) {
            // we got a rising edge! Now, which direction are we going?
            if (GPIO_ReadPinInput(GPIOB, ENC_RIGHT_B)) {
                enc_admin.right_count += 1;
            } else {
                enc_admin.right_count -= 1;
            }

        } else {
            // we got a falling edge! Now, which direction are we going?
            if (GPIO_ReadPinInput(GPIOB, ENC_RIGHT_B)) {
                enc_admin.right_count -= 1;
            } else {
                enc_admin.right_count += 1;
            }
        }

    } else if(pin_mask & ENC_RIGHT_B_MASK) {
        // encoder B has changed. High or low?
        if (GPIO_ReadPinInput(GPIOB, ENC_RIGHT_B)) {
            // we got a rising edge! Now, which direction are we going?
            if (GPIO_ReadPinInput(GPIOB, ENC_RIGHT_A)) {
                enc_admin.right_count -= 1;
            } else {
                enc_admin.right_count += 1;
            }

        } else {
            // we got a falling edge! Now, which direction are we going?
            if (GPIO_ReadPinInput(GPIOB, ENC_RIGHT_A)) {
                enc_admin.right_count += 1;
            } else {
                enc_admin.right_count -= 1;
            }
        }


    /* --- LEFT MOTOR CHECK --- */
    } else if (pin_mask & ENC_LEFT_A_MASK) {
        // encoder A has changed. High or low?
        if (GPIO_ReadPinInput(GPIOB, ENC_LEFT_A)) {
            // we got a rising edge! Now, which direction are we going?
            if (GPIO_ReadPinInput(GPIOB, ENC_LEFT_B)) {
                enc_admin.right_count += 1;
            } else {
                enc_admin.right_count -= 1;
            }

        } else {
            // we got a falling edge! Now, which direction are we going?
            if (GPIO_ReadPinInput(GPIOB, ENC_LEFT_B)) {
                enc_admin.right_count -= 1;
            } else {
                enc_admin.right_count += 1;
            }
        }

    } else if(pin_mask & ENC_LEFT_B_MASK) {
        // encoder B has changed. High or low?
        if (GPIO_ReadPinInput(GPIOB, ENC_LEFT_B)) {
            // we got a rising edge! Now, which direction are we going?
            if (GPIO_ReadPinInput(GPIOB, ENC_LEFT_A)) {
                enc_admin.right_count -= 1;
            } else {
                enc_admin.right_count += 1;
            }

        } else {
            // we got a falling edge! Now, which direction are we going?
            if (GPIO_ReadPinInput(GPIOB, ENC_LEFT_A)) {
                enc_admin.right_count += 1;
            } else {
                enc_admin.right_count -= 1;
            }
        }

    /* NOT GOOD. Some other pin interrupted us... */
    } else {
        assert(false);
    }

    // clear the pins with interrupt flags
    GPIO_ClearPinsInterruptFlags(GPIOB, pin_mask);
}
