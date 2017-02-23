/*
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*
* o Redistributions of source code must retain the above copyright notice, this list
*   of conditions and the following disclaimer.
*
* o Redistributions in binary form must reproduce the above copyright notice, this
*   list of conditions and the following disclaimer in the documentation and/or
*   other materials provided with the distribution.
*
* o Neither the name of Freescale Semiconductor, Inc. nor the names of its
*   contributors may be used to endorse or promote products derived from this
*   software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
* ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "hardware.h"
#include "fsl_port.h"


/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/* initialize GPIO pins and setup hardware modules */
void hw_init_pins(void)
{
    /* Ungate the port clock */
    CLOCK_EnableClock(kCLOCK_PortA);
    CLOCK_EnableClock(kCLOCK_PortB);
    // Setup PORTB
    PORT_SetPinMux(PORTB, 0u, kPORT_PinDisabledOrAnalog); // FB_R - Analog read of the right motor (ADC0_SE9 by default)
    PORT_SetPinMux(PORTB, 1u, kPORT_MuxAsGpio);           // right_encA - Digital square wave to read encoder distance... (alt 1)
    PORT_SetPinMux(PORTB, 2u, kPORT_MuxAsGpio);           // right_encB - Digital square wave to read encoder distance... (alt 1)
    PORT_SetPinMux(PORTB, 3u, kPORT_MuxAlt2);             // I2C_SCL - I2C Clock (alt 2)
    PORT_SetPinMux(PORTB, 4u, kPORT_MuxAlt2);             // I2C_SDA - I2C Data (alt 2)
    // JTAG_SWO - JTAG SWO pin (N/A)
    PORT_SetPinMux(PORTB, 6u, kPORT_MuxAsGpio);           // left_encA - Digital square wave to read encoder distance... (alt 1)
    PORT_SetPinMux(PORTB, 7u, kPORT_MuxAsGpio);           // left_encB - Digital square wave to read encoder distance... (alt 1)
    PORT_SetPinMux(PORTB, 10u, kPORT_MuxAlt2);            // MotorL_D_n - Disables the left motor. Can PWM for speed control (TPM0_CH1 - alt 2)
    PORT_SetPinMux(PORTB, 11u, kPORT_MuxAsGpio);          // MotorL_Inv - Invert the left motors direction (alt 1)
    PORT_SetPinMux(PORTB, 13u, kPORT_MuxAsGpio);          // I2C_LSB_BIT - Controls the I2C address least significant bit (alt 1)

    // JTAG_SWCLK - JTAG SWCLK pin (N/A)
    // JTAG_RESET - JTAG RESET pin (N/A)
    // JTAG_SWDIO - JTAG SWDIO pin (N/A)
    PORT_SetPinMux(PORTA, 3u, kPORT_MuxAlt4);             // rear_bumper - rear bumper sensor   (alt 1) / Debug UART-TX (alt 4)
    PORT_SetPinMux(PORTA, 4u, kPORT_MuxAlt4);             // front_bumper - front bumper sensor (alt 1) / Debug UART-RX (alt 4)
    PORT_SetPinMux(PORTA, 5u, kPORT_MuxAsGpio);           // MotorSF_n - Motor status flag to indicate a motor driver error (alt 1)
    PORT_SetPinMux(PORTA, 6u, kPORT_MuxAsGpio);           // Motor_In1 - Motor driver input (alt 1)
    PORT_SetPinMux(PORTA, 7u, kPORT_MuxAsGpio);           // Motor_In2 - Motor driver input (alt 1)
    PORT_SetPinMux(PORTA, 8u, kPORT_PinDisabledOrAnalog); // FB_L - Analog read of the left motor (ADC0_SE3 by default)
    PORT_SetPinMux(PORTA, 9u, kPORT_PinDisabledOrAnalog); // VBATT_MONITOR - Analog read of the motor voltage. (ADC0_SE2 by default)
    PORT_SetPinMux(PORTA, 12u, kPORT_MuxAlt2);            // MotorR_D_n - Disables the right motor. Can PWM for speed control (TPM1_CH0 = alt 2)
}
