/*
 * Encoder handler functions and definitions.
 *
 * Author: Tyler Holmes (tyler@holmesengineering.com)
 * Date: 1/2/2016
 */

#ifndef _ENCODERS_H_
#define _ENCODERS_H_

/* required headers for this header */
#include <stdint.h>

/* Public Data Structures */

//! Encoder type to select from.
typedef enum {
    kEncoder_Left,
    kEncoder_Right
} enc_type_t;

/* Public Function Definitions */

//! Initializes the encoders admin structure and sets up the pin interrupts.
void encoders_init(void);

/*!
 * @brief Gets the current encoder count from the given encoder type.
 *
 * @param encoder_type (enc_type_t): Encoder you want to retrieve from.
 * @return (int32_t): Current encoder count from `encoder_type`.
 */
int32_t encoders_get_counts(enc_type_t encoder_type);

/*!
 * @brief Resets the given encoder type's count to zero.
 *
 * @param encoder_type (enc_type_t): Encoder you want to reset.
 */
void encoders_reset_counts(enc_type_t encoder_type);

#endif /* _ENCODERS_H_ */
