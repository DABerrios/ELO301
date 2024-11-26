
/**
 ******************************************************************************
 * @file    controler.h
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
#ifndef INC_CONTROLER_H_
#define INC_CONTROLER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "tim.h"

/*- PRIVATE_Definitions ------------------------------------------------------*/

/*- PRIVATE_Types ------------------------------------------------------------*/
/**
 *
 */

typedef struct
{
  real_t derState;      // Last position input
  real_t integratState; // Integrator state
  real_t integratMax,    // Maximum and minimum
  integratMin;          // allowable integrator state
  real_t integratGain, // integral gain
  propGain,            // proportional gain
  derGain;             // derivative gain
} SPid;


/*- PUBLIC_API ---------------------------------------------------------------*/
void StabilizeMotor(MotorState *state, SPid *pid, uint8_t *pwmOutput1, uint8_t *pwmOutput2);
real_t UpdatePID(SPid * pid, real_t error, real_t position);

#ifdef __cplusplus
}
#endif
#endif /* INC_CONTROLER_H_ */
