/**
 ******************************************************************************
 * @file    lsm6ds3.h
 * @author  ELO301
 * @date    Dec 1, 2022
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
#ifndef _ELO301_INC_LSM6_H_
#define _ELO301_INC_LSM6_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>


/*- USER_Definitions ------------------------------------------------------*/
#include "main.h"

#define LSM6DS3_I2C_ADDR    0x6B<<1
#define LSM6DS3_WHO_AM_I    0x0F
#define LSM6DS3_CTRL1_XL    0x10
#define LSM6DS3_STATUS_REG	0x1E
#define LSM6DS3_X_OFS_USR	0x73
#define LSM6DS3_Y_OFS_USR	0x74
#define LSM6DS3_Z_OFS_USR	0x75
#define LSM6DS3_CTRL7_G		0x16


#define LSM6DS3_OUTX_L_XL	0x28
#define LSM6DS3_OUTX_H_XL	0x29
#define LSM6DS3_OUTY_L_XL	0x2A
#define LSM6DS3_OUTY_H_XL	0x2B
#define LSM6DS3_OUTZ_L_XL	0x2C
#define LSM6DS3_OUTZ_H_XL	0x2D
/*- PRIVATE_Definitions ------------------------------------------------------*/


/*- PRIVATE_Types ------------------------------------------------------------*/
/**
 * 
 */
typedef struct
{
} t_lsm6ds3;


/*- PUBLIC_API ---------------------------------------------------------------*/

void lsm6ds3_init(void);

bool lsm6ds3_open(void);

bool lsm6ds3_update(void);
void lsm6ds3_read_accelerometer(accel_data **data);
void lsm6ds3_accelerometer_mode(void);

#ifdef __cplusplus
}
#endif

#endif   // _ELO301_INC_PWM_H_
