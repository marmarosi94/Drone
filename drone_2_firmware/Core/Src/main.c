/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

// Global variables to track time

/* USER CODE BEGIN PV */

/* Private variables ---------------------------------------------------------*/
char str[UART_TX_BUFFER_SIZE] = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

/**
  * @brief  The application entry point.
  * @retval int
  */

int main(void)
{

	/* USER CODE BEGIN 1 */
    uint32_t t1 = 0;
	uint32_t t2 = 0;
	uint32_t t_opt = 0;
	uint32_t t3 = 0;
	uint32_t t_imu = 0;
	uint32_t t_tof = 0;

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();
	/* Configure the system clock */
	SystemClock_Config();
	/* USER CODE BEGIN 2 */
	DWT_Init();
	MX_GPIO_Init();
	comm_init();
	debug_print("Wizard booted!!!\r\n");
	timers_init();
	esc_init();
	HAL_TIM_Base_Start(&htim2);	//Start timer for inits
	IMU_Init();
	VL53L1X_SensorInit(tof_dev);
	PID_Init();
	IMU_Calib();
	PMW3901_Init();
	MX_SPI2_Init_AFTER_OPTIC();

	/* USER CODE END 2 */
	t1 = t_imu = t_tof = t2 = t_opt = t3 = pid_lasttime = get_us();	//Because many cycle is already happened and get_us() will return with big numbers of cycles
	/* USER CODE BEGIN WHILE */
	while (1)
	{
	  uint32_t now = DWT->CYCCNT;

	  if ((now - t1) >= LOOP0_5_CYCLES)
	  {
		  I2C_Task();
		  t1 += LOOP0_5_CYCLES;
	  }
	  if(now - t_imu >= IMU_CYCLES)
	  {
		  IMU_Task();
          IMU_compute_rotation();
          accel_to_wframe(accel_body, &accel_world);
		  t_imu += IMU_CYCLES;
	  }

	  if(now - t_tof >= TOF_CYCLES)
	  {
		  TOF_Task_Status();
		  t_tof += TOF_CYCLES;
	  }

	  if ((now - t_opt) >= OPTICAL_CYCLES)
	  {
		  optical_request_motion();
		  t_opt += OPTICAL_CYCLES;
	  }

	  if ((now - t2) >= LOOP2_CYCLES)
	  {
		  pid_deltatime = pid_deltatime_us() * INV_CPU_FREQ;

		  euler_flt = quat_to_euler(quat_flt_orientation);

		  if(spi_state == SPI_STATE_DONE)
		  {
			  //optical_parse(spi_buff);
			  spi_state = SPI_STATE_IDLE;
		  }

		  pid_traget.Roll  = compute_pid(&pd_pos, 0.0f, horizontal_state.x, horizontal_state.vy, pid_deltatime);
		  pid_traget.Pitch = compute_pid(&pd_pos, 0.0f, horizontal_state.x, horizontal_state.vx, pid_deltatime);
		  pid_traget.Yaw = 0;
		  pid_traget.Throttle = compute_pid(&pd_height, 420.0f, vertical_state.z, vertical_state.vz, pid_deltatime);

		  pid_control.Roll   = compute_pid(&pid_roll, pid_traget.Roll, euler_flt.roll, gyro_frame_deg.y, pid_deltatime);
		  pid_control.Pitch  = compute_pid(&pid_pitch, pid_traget.Pitch, euler_flt.pitch, gyro_frame_deg.x, pid_deltatime);
		  pid_control.Yaw    = compute_pid(&pid_yaw, pid_traget.Yaw, euler_flt.yaw, gyro_frame_deg.z, pid_deltatime);
		  pid_control.Throttle = 1190 + pid_traget.Throttle;

		  update_motors(pid_control.Throttle, pid_control.Roll, pid_control.Pitch, pid_control.Yaw);

		  t2 += LOOP2_CYCLES;
	  }

	  if ((now - t3) >= LOOP10MS_CYCLES)
	  {
		  //flight_statemachine();

		  //pid_control.Throttle = 1150;
/*
    	  sprintf(str,"position.x: %f, position.y: %f, position.z: %f\r\n", position.x, position.y, position.z);
		  debug_print(str);

    	  sprintf(str,"Roll: %f, Pitch: %f, Yaw: %f\r\n", euler_flt.roll, euler_flt.pitch, euler_flt.yaw);
		  debug_print(str);
*/
    	  sprintf(str,"pid_traget.Roll: %f, pid_traget.Pitch: %f, pid_traget.Throttle: %d\r\n", pid_traget.Roll, pid_traget.Pitch, pid_traget.Throttle);
		  debug_print(str);
/*
	 	  sprintf(str,"pid_control.Roll: %f, pid_control.Pitch: %f, pid_control.YAW: %f, pid_control.Throttle: %d\r\n", pid_control.Roll, pid_control.Pitch, pid_control.Yaw, pid_control.Throttle);
	 	  debug_print(str);
*/
		  ekf_vertical();
		  ekf_horzintal();
		  t3 += LOOP10MS_CYCLES;
	  }
	  SENSOR_Process();
	  //optical_parse(spi_buff);
	}
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

	/*Configure GPIO pin : PB12 */
	GPIO_InitStruct.Pin = GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

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
