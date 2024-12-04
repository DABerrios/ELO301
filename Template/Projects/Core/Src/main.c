/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* FOO */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "gpio_if.h"
#include "adc_if.h"
#include "pwm.h"
#include "lsm6ds3.h"
#include "encoder.h"
#include "controler.h"

  MotorState motorState;
  motor_data motordata;
  MotorState* motorS;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TX_TIMEOUT_MS  100   /**< Transmission time timeout over UART */
#define DELAY_MS       100   /**< Delay timeout */
#define GPIO_LD2_ENABLED 0 /**< Use LD2 as GPIO driven LED. It needs special config in IOC */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
#if GPIO_LD2_ENABLED
  t_gpio_pin user_led_pin = {LD4_GPIO_Port, LD4_Pin};
  t_gpio_if user_led;
#endif
  t_gpio_pin user_button_pin = {B1_GPIO_Port, B1_Pin};
  t_gpio_if user_button;
  t_adc_if potentiometer;
  t_pwm PWM1;
  t_pwm PWM2;
  uint8_t adc_rate;
  t_encoder encoder;
  uint8_t pwmOutput1 = 0;
  uint8_t pwmOutput2 = 0;
  uint8_t* pwmOutput1_ptr = &pwmOutput1;
  uint8_t* pwmOutput2_ptr = &pwmOutput2;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  /* Init custom GPIO */
#if GPIO_LD2_ENABLED
  gpio_if_init(&user_led, ACTIVE_HIGH, &user_led_pin, GPIO_IF_CLEAR);
  if (gpio_if_open(&user_led) != GPIO_IF_SUCCESS)
  {
    Error_Handler();
  }
#endif
  gpio_if_init(&user_button, ACTIVE_LOW, &user_button_pin, GPIO_IF_INPUT);
  if (gpio_if_open(&user_button) != GPIO_IF_SUCCESS)
  {
    Error_Handler();
  }

  /* Init custom ADC */
  adc_if_init(&potentiometer, &hadc1);
  if (adc_if_open(&potentiometer) != ADC_IF_SUCCESS)
  {
    Error_Handler();
  }

  SPid pid = {
      .derState = 0,
      .integratState = 0,
      .integratMax = 100,
      .integratMin = -100,
      .integratGain = 0.01,
      .propGain = 0.25,
      .derGain = 0.001
  };
  /* Init PWM */
  pwm_init(&PWM1, &htim2, TIM_CHANNEL_1, COUNTER_PERIOD_VALUE);
  if (pwm_open(&PWM1) != PWM_SUCCESS)
  {
    Error_Handler();
  }

  pwm_init(&PWM2, &htim2, TIM_CHANNEL_2, COUNTER_PERIOD_VALUE);
	if (pwm_open(&PWM2) != PWM_SUCCESS)
	{
	  Error_Handler();
	}
  encoder_init(&encoder, &htim1, TIM_CHANNEL_ALL, COUNTER_PERIOD_VALUE);
  if (encoder_open(&encoder) != ENCODER_SUCCESS) {
    Error_Handler();
  }

  //motor_data motordata;
  motor_data* motor=&motordata;
  //MotorState motorState;
  motorS=&motorState;
  motorS->target = 0;
  encoder_start(&encoder);
  __HAL_TIM_SET_COUNTER(&htim1, 32000);
  encoder_read(&encoder, &motor->position, &motor->direction);
  //printf("Encoder position: %lu\r\n", motor->position);
  //printf("Encoder direction: %lu\r\n", motor->direction);
  //encoder_to_degrees(motor->position);
  //printf("Encoder position in degrees: %f\r\n", encoder_to_degrees(motor->position));
  lsm6ds3_init();

  /* Welcome message */
  //printf("ELO301 Demo Init\r\n");
  lsm6ds3_accelerometer_mode();
  accel_data acceldata;
  accel_data* data=&acceldata;
  lsm6ds3_read_accelerometer(&data);


  while (1)
  {
#if GPIO_LD2_ENABLED
    /* Blink user LED */
    gpio_if_toggle(&user_led);
#endif
    if (adc_if_get_rate(&potentiometer, &adc_rate) != ADC_IF_SUCCESS)
    {
      Error_Handler();
    }

    lsm6ds3_read_accelerometer(&data);
    motorS->tilt = lsm6ds3_g_to_degrees(data->x, data->y, data->z);
    encoder_read(&encoder, &motor->position, &motor->direction);
    motorS->position = encoder_to_degrees(motor->position);
    motorS->target = -motorS->tilt + 180.0 + adc_rate*180;

    StabilizeMotor(motorS, &pid, pwmOutput1_ptr, pwmOutput2_ptr);
    pwm_update(&PWM1, pwmOutput1);
    pwm_update(&PWM2, pwmOutput2);
    // Turn IN1_motor on
    HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
    
    HAL_Delay(DELAY_MS);

    /* Print message */
    //printf("Button: %u\r\n", gpio_if_get(&user_button));

    /* Read LSM6DS3 driver */
    /*if (lsm6ds3_update())
    {
      printf( "LSM6DS3 updated\r\n" );
    }
    lsm6ds3_read_accelerometer(&data);
    //printf("Accelerometer:\n x: %G\n y: %G\n z: %G\r\n", data->x, data->y, data->z);
    printf("%G\t%G\t%G\r\n", data->x, data->y, data->z);
    */
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
 * @brief Function that bridges HAL with printf
 * @param file Not in use
 * @param ptr Pointer to data to be printed out
 * @param len Length of the data to be printed out
 * @return len
 */
int _write(int file, char *ptr, int len)
{
  if (HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, TX_TIMEOUT_MS) != HAL_OK)
  {
    Error_Handler();
  }

  return len;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
