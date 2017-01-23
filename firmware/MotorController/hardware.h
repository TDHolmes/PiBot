#ifndef _HARDWARE_H_
#define _HARDWARE_H_

#include "clock_config.h"
#include "fsl_gpio.h"
#include "hardware.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief The board name */
#define HW_NAME "PYCO-BOT-MOTOR-DRIVER"


/*******************************************************************************
 * Public Function Definitions
 ******************************************************************************/

void hw_init_pins(void);
#endif /* _HARDWARE_H_ */
