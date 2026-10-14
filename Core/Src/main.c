/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (Nucleo-F410RB Cleaned)
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "motor_control.h"
#include "gpio.h"
#include "tim.h"
#include "i2c.h"
#include <stdio.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
volatile uint16_t encoder_raw_value = 0;
volatile HAL_StatusTypeDef i2c_status = HAL_OK;
volatile uint8_t magnet_status = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  /* Initialize peripherals from gpio.c, tim.c, i2c.c */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */
  // 1. Start PWM Output (PA8)
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  // 2. Start Quadrature Encoder (PA0/PA1)
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);

  // 3. Start 10ms Control Loop Interrupt
  HAL_TIM_Base_Start_IT(&htim6);

  // 4. Initialize Motor Control Subsystems
  MotorControl_Init();
  Elbow_Control_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      HAL_Delay(50);
  }
  /* USER CODE END WHILE */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) // 10ms Interrupt
    {
        MotorControl_Update(0.01f);

        // Disabled: prevents elbowMotor from overwriting PA8 PWM and PB4 direction pin
        // float target_pos = 2000.0f;
        // Elbow_Control_Update(target_pos, 0.01f);
    }
}
/* USER CODE END 4 */

/**
  * @brief System Clock Configuration
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
