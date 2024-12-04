/**
 ******************************************************************************
 * @file    lsm6ds3.c
 * @author  ELO301
 * @date    Dec 1, 2022
 * @brief   Driver for LSM6DS3 IMU sensor
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
#include "lsm6ds3.h"
#include "main.h"
#include "i2c.h"
#include "math.h"

/*- PRIVATE_TUNABLES ---------------------------------------------------------*/

/*- PRIVATE_Definitions ------------------------------------------------------*/
#define TIMEOUT_100MS 100

/*- PRIVATE_Macros -----------------------------------------------------------*/

/*- PRIVATE_Types ------------------------------------------------------------*/

/*- PRIVATE_Functions --------------------------------------------------------*/

/*- PRIVATE_Data -------------------------------------------------------------*/

/*- PUBLIC_API ---------------------------------------------------------------*/
/*
 * API: pwm_init
 */
void lsm6ds3_init(void)
{
  uint8_t who_am_i = 0;
  HAL_StatusTypeDef status;

  /* Test I2C interface by reading known value (WHO AM I) using HAL interface */
  status = HAL_I2C_Mem_Read(&hi2c1, LSM6DS3_I2C_ADDR, LSM6DS3_WHO_AM_I, 1, &who_am_i, 1, TIMEOUT_100MS);
  if (status != HAL_OK)
  {
    /* I2C error */
    Error_Handler();
    return;
  }
}
void lsm6ds3_accelerometer_mode(void) {
	HAL_StatusTypeDef status;
	uint8_t ctrl1xl=0x4C;//accelerometer normal mode 104 Hz +-8g
	status = HAL_I2C_Mem_Write(&hi2c1, LSM6DS3_I2C_ADDR, LSM6DS3_CTRL1_XL, 1, &ctrl1xl, 1,TIMEOUT_100MS);
	if (status != HAL_OK)
	  {
	    /* I2C error */
	    Error_Handler();
	    return;
	  }
	return;
}
void lsm6ds3_accelerometer_offset(void){
	HAL_StatusTypeDef status;
	uint8_t USR_OFF_ON_OUT = 0x02;
	uint8_t offsetxl = 0x00;
	status= HAL_I2C_Mem_Write(&hi2c1, LSM6DS3_I2C_ADDR, LSM6DS3_CTRL7_G, 1, &USR_OFF_ON_OUT, 1, TIMEOUT_100MS);
	if (status != HAL_OK) {
		/* I2C error */
		Error_Handler();
		return;
	}
	status = HAL_I2C_Mem_Write(&hi2c1, LSM6DS3_I2C_ADDR, LSM6DS3_X_OFS_USR, 1,
			&offsetxl, 1, TIMEOUT_100MS);
	if (status != HAL_OK) {
		/* I2C error */
		Error_Handler();
		return;
	}
	status = HAL_I2C_Mem_Write(&hi2c1, LSM6DS3_I2C_ADDR, LSM6DS3_Y_OFS_USR, 1,
			&offsetxl, 1, TIMEOUT_100MS);
	if (status != HAL_OK) {
		/* I2C error */
		Error_Handler();
		return;
	}
	status = HAL_I2C_Mem_Write(&hi2c1, LSM6DS3_I2C_ADDR, LSM6DS3_Z_OFS_USR, 1,
			&offsetxl, 1, TIMEOUT_100MS);
	if (status != HAL_OK) {
		/* I2C error */
		Error_Handler();
		return;
	}
	return;
}

void lsm6ds3_read_accelerometer(accel_data **data) {
	HAL_StatusTypeDef status;
	uint8_t xldata;
	int16_t xacceleration;
	int16_t yacceleration;
	int16_t zacceleration;
	uint8_t xacceleration2;
	uint8_t yacceleration2;
	uint8_t zacceleration2;
	status = HAL_I2C_Mem_Read(&hi2c1, LSM6DS3_I2C_ADDR, LSM6DS3_STATUS_REG, 1, &xldata, 1, TIMEOUT_100MS);
	if (status != HAL_OK) {
		/* I2C error */
		Error_Handler();
		return;
	}
	xldata = xldata & 0x01;
	if (xldata == 0) {
		return;
	}
	else {
		status = HAL_I2C_Mem_Read(&hi2c1, LSM6DS3_I2C_ADDR, LSM6DS3_OUTX_H_XL, 1, &xacceleration2, 1, TIMEOUT_100MS);
		if (status != HAL_OK) {
			/* I2C error */
			Error_Handler();
			return;
		}

		xacceleration = xacceleration2 << 8;
		status = HAL_I2C_Mem_Read(&hi2c1, LSM6DS3_I2C_ADDR, LSM6DS3_OUTX_L_XL, 1, &xacceleration2, 1, TIMEOUT_100MS);
		if (status != HAL_OK) {
			/* I2C error */
			Error_Handler();
			return;
		}
		xacceleration = xacceleration | xacceleration2;
		status = HAL_I2C_Mem_Read(&hi2c1, LSM6DS3_I2C_ADDR, LSM6DS3_OUTY_H_XL, 1, &yacceleration2, 1, TIMEOUT_100MS);
		if (status != HAL_OK) {
			/* I2C error */
			Error_Handler();
			return;
		}
		yacceleration = yacceleration2 << 8;
		status = HAL_I2C_Mem_Read(&hi2c1, LSM6DS3_I2C_ADDR, LSM6DS3_OUTY_L_XL, 1, &yacceleration2, 1, TIMEOUT_100MS);
		if (status != HAL_OK) {
			/* I2C error */
			Error_Handler();
			return;
		}
		yacceleration = yacceleration | yacceleration2;
		status = HAL_I2C_Mem_Read(&hi2c1, LSM6DS3_I2C_ADDR, LSM6DS3_OUTZ_H_XL, 1, &zacceleration2, 1, TIMEOUT_100MS);
		if (status != HAL_OK) {
			/* I2C error */
			Error_Handler();
			return;
		}
		zacceleration = zacceleration2 << 8;
		status = HAL_I2C_Mem_Read(&hi2c1, LSM6DS3_I2C_ADDR, LSM6DS3_OUTZ_L_XL, 1, &zacceleration2, 1, TIMEOUT_100MS);
		if (status != HAL_OK) {
			/* I2C error */
			Error_Handler();
			return;
		}
		zacceleration = zacceleration | zacceleration2;
		(*data)->x = xacceleration * 0.244/1000;
		(*data)->y = yacceleration * 0.244/1000;
		(*data)->z = zacceleration * 0.244/1000;
	    return;
	}
}
float lsm6ds3_g_to_degrees(float gx, float gy, float gz) {
	float degrees = atan2(gy , sqrt(pow(gx,2)+pow(gz,2))) * 180 / 3.14159265359;
	return degrees;
}
/*
 * API: pwm_open
 */
bool lsm6ds3_open(void)
{
  return false;
}

bool lsm6ds3_update(void)
{
  return false;
}

/*- PRIVATE_Functions --------------------------------------------------------*/
