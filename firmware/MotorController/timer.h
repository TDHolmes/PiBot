/*
 * Timer setup and run functions
 *
 * Author: Tyler Holmes (tyler@holmesengineering.com)
 * Date: 1/21/2016
 */

#ifndef _TIMER_H_
#define _TIMER_H_

#include <stdint.h>


/*******************************************************************************
 * Public Data Structures
 ******************************************************************************/


/*******************************************************************************
 * Public Function Definitions
 ******************************************************************************/

uint32_t timer_get_tick(void);
void timer_clear_tick(void);
void timer_init(uint8_t * flag_ptr, uint16_t flag_threshold);


#endif /* _TIMER_H_ */
