/**
 ******************************************************************************
 * @file    controler.c
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
#include <math.h>
#include "controler.h"
/*- PRIVATE_TUNABLES ---------------------------------------------------------*/

/*- PRIVATE_Definitions ------------------------------------------------------*/



/*- PRIVATE_Macros -----------------------------------------------------------*/

/*- PRIVATE_Types ------------------------------------------------------------*/

/*- PRIVATE_Functions --------------------------------------------------------*/

/*- PRIVATE_Data -------------------------------------------------------------*/

/*- PUBLIC_API ---------------------------------------------------------------*/


 uint32_t UpdatePID(SPid * pid, uint32_t error, uint32_t position)
{
  uint32_t pTerm, dTerm, iTerm;
  pTerm = pid->propGain * error; // calculate the proportional term
  // calculate the integral state with appropriate limiting
  pid->integratState += error;
  // Limit the integrator state if necessary
  if (pid->integratState > pid->integratMax)
  {
    pid->integratState = pid->integratMax;
  }
  else if (pid->integratState < pid->integratMin)
  {
    pid->integratState = pid->integratMin;
  }
  // calculate the integral term

  iTerm = pid->integratGain * pid->integratState;
  // calculate the derivative
  dTerm = pid->derGain * (pid->derState - position);
  pid->derState = position;
  return pTerm + dTerm + iTerm;
}
void StabilizeMotor(MotorState *state, SPid *pid, uint8_t *pwmOutput1, uint8_t *pwmOutput2)
{
	uint32_t tiltError = state->tilt;
	uint32_t posError = state->target - state->position;

	uint32_t totalError = tiltError + posError;

	uint32_t controlSignal = UpdatePID(pid, totalError, state->position);

	uint32_t scaledSignal = fabs(controlSignal);
  if (scaledSignal > 100) {
    scaledSignal = 100;
  }
  uint8_t pwmSignal = (uint8_t)scaledSignal;

  if (controlSignal > 0) {
    *pwmOutput1 = pwmSignal;
    *pwmOutput2 = 0;
  } else {
    *pwmOutput1 = 0;
    *pwmOutput2 = -pwmSignal;
  }


}

