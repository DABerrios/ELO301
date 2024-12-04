/**
 ******************************************************************************
 * @file    encoder.c
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

#include <stdlib.h>
#include "encoder.h"

/*- PRIVATE_TUNABLES ---------------------------------------------------------*/

/*- PRIVATE_Definitions ------------------------------------------------------*/

/*- PRIVATE_Macros -----------------------------------------------------------*/

/*- PRIVATE_Types ------------------------------------------------------------*/

/*- PRIVATE_Functions --------------------------------------------------------*/

/*- PRIVATE_Data -------------------------------------------------------------*/

/*- PUBLIC_API ---------------------------------------------------------------*/

void encoder_init(t_encoder *encoder, TIM_HandleTypeDef *tim, uint32_t channel, uint16_t counter_period_value)
{
  encoder->tim = tim;
  encoder->channel = channel;
  encoder->counter_period_value = counter_period_value;
}
t_encoder_status encoder_open(t_encoder *encoder)
{
  if (HAL_TIM_Encoder_Init(encoder->tim, encoder->channel) != HAL_OK) {
    return ENCODER_ERROR;
  }

  return ENCODER_SUCCESS;
}
t_encoder_status encoder_start(t_encoder *encoder) {
	if (HAL_TIM_Encoder_Start(encoder->tim, encoder->channel) != HAL_OK) {
		return ENCODER_ERROR;
	}

	return ENCODER_SUCCESS;
}
t_encoder_status encoder_read(t_encoder *encoder, uint32_t *value, uint32_t *direction)
{
  *value = encoder->tim->Instance->CNT;
  *direction = encoder->tim->Instance->CR1;
  return ENCODER_SUCCESS;
}
float encoder_to_degrees(uint32_t value)
 {
  float revolutions = (float)value;
  revolutions = revolutions/(float)(ENCODER_GEAR_RATIO*ENCODER_PULSES_PER_REVOLUTION);
  //float degrees = fmod(revolutions * 360.0, 360.0);
  float degrees = revolutions * 360.0;

  return degrees;
  }




  /* Init variables */

  /* Start peripherals */


/*- PRIVATE_Functions --------------------------------------------------------*/


