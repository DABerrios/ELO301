
/**
 ******************************************************************************
 * @file    encoder.h
 * @author  Daisy Berrios
 * @date    Nov 19, 2024
 * @brief
 *
 *
 * @note
 * @warning
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2022 ELO301</center></h2>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 ******************************************************************************
 */
#ifndef ELO301_INC_ENCODER_H_
#define ELO301_INC_ENCODER_H_
#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "tim.h"

/*- PRIVATE_Definitions ------------------------------------------------------*/
#define ENCODER_PULSE_PER_REVOLUTION 28

/*- PRIVATE_Types ------------------------------------------------------------*/
typedef struct {
	TIM_HandleTypeDef *tim;
	uint32_t channel;
	uint16_t counter_period_value;
} t_encoder;

typedef enum {
	ENCODER_SUCCESS,
	ENCODER_ERROR
} t_encoder_status;

/*- PUBLIC_API ---------------------------------------------------------------*/

void encoder_init(t_encoder *encoder, TIM_HandleTypeDef *tim, uint32_t channel, uint16_t counter_period_value);
t_encoder_status encoder_open(t_encoder *encoder);
t_encoder_status encoder_read(t_encoder *encoder, uint32_t *value, uint32_t *direction);
float encoder_to_degrees(uint32_t value);



#endif /* ELO301_INC_ENCODER_H_ */
