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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define UART_TX_BUFFER_SIZE 1024
#define UART_RX_BUFFER_SIZE 32
#define I2C_TX_BUFFER_SIZE 32
#define I2C_RX_BUFFER_SIZE 32

#define IMU_I2C_ADDRESS 0xD0 // Example I2C address of the IMU (MPU6050) - 7-bit: 0x68
// Registers for configuration
#define PWR_MGMT_1_REG  0x6B
#define CONFIG_REG      0x1A
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define SMPLRT_DIV_REG  0x19

// Global variables to track time
volatile uint32_t millis;  // Millisecond counter
volatile uint32_t timeout_flag;  // Timeout flag for time-based events

char uart1_txBuffer[UART_TX_BUFFER_SIZE];  // Transmit buffer
char uart1_rxBuffer[UART_RX_BUFFER_SIZE];  // Receive buffer (if needed)

volatile uint16_t txWriteIndex = 0;
volatile uint16_t txReadIndex = 0;
volatile uint8_t txBusy = 0;
char str_tmp[UART_TX_BUFFER_SIZE] = {0};

uint8_t i2c1_txBuffer[I2C_TX_BUFFER_SIZE];
uint8_t i2c1_rxBuffer[I2C_RX_BUFFER_SIZE];
static uint8_t dma_read_complete = 0;  // Flag to indicate DMA read completion

uint8_t imu_data[14];  // Array to store accelerometer and gyroscope data (12 bytes)

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_tx;
DMA_HandleTypeDef hdma_i2c1_rx;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
uint32_t get_millis(void);
void delay(uint32_t ms);
void USART1_Transmit_DMA(char *data, uint16_t size);
int timeout(uint32_t start_time, uint32_t timeout_period);
HAL_StatusTypeDef I2C_Write_DMA(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t size);
HAL_StatusTypeDef I2C_Read_DMA(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t size);
void debug_print(char* str);
void IMU_Init(void);
void IMU_Read_Accel_Gyro(void);
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim6);
  debug_print("Wizard booted!!!\r\n");
  IMU_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //sprintf(str_tmp, "ACCEL_X:0\r\n");
	  //USART1_Transmit_DMA(str_tmp, strlen(str_tmp));

	  IMU_Read_Accel_Gyro();
	  delay(10);
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x0010020A;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 6400;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 10;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 460800;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Function to initialize the IMU
// Function to initialize the IMU
void IMU_Init(void) {
    uint8_t data[2];
    char str[64];

    // Wake up MPU6050 (write 0x00 to PWR_MGMT_1 register)
    data[0] = 0x00;   // Value to wake up the IMU
    if (I2C_Write_DMA(IMU_I2C_ADDRESS, 0x6B, data, 1) != HAL_OK) {
        debug_print("Error: Failed to wake up IMU\r\n");
        return;  // Exit if I2C write failed
    }

    delay(100);  // Need 100ms after the reset for the IMU

    uint8_t i = 0;
    uint32_t start_time = get_millis();
    uint32_t timeout_duration = 1000;  // Timeout after 1000 ms (1 second)
    uint32_t retry_delay = 50;  // Delay between retry attempts (in ms)

    // Loop through retry attempts for IMU detection, prioritizing retries
    while (i < 10) {  // Max 10 attempts, retry-focused
        // Clear the DMA completion flag before starting a new read
        dma_read_complete = 0;

        // Start I2C read with DMA (non-blocking call)
        I2C_Read_DMA(IMU_I2C_ADDRESS, 0x75, data, 1);

        // Wait for DMA read to complete with timeout logic
        uint32_t wait_start_time = get_millis();
        while (!dma_read_complete) {
            // If the overall timeout period has passed, exit with an error
            if (get_millis() - start_time > timeout_duration) {
                debug_print("Error: IMU not responding within timeout period\r\n");
                return;  // Exit if overall timeout has been exceeded
            }

            // If retry delay threshold has passed, break the inner loop and retry
            if (get_millis() - wait_start_time > retry_delay) {
                break;  // Exit the current wait loop and try again
            }
        }

        // Check if the IMU is responding correctly (should return 0x68)
        if (data[0] == 0x68) {
            debug_print("IMU FOUND!\r\n");
            return;  // Successfully found the IMU, exit the function
        } else {
            sprintf(str, "IMU NOT found, attempt %d\r\n", i);
            debug_print(str);  // Print attempt message
        }

        // Increment retry attempt
        i++;

        // Add a small delay before retrying (to prevent hammering)
        HAL_Delay(10);  // 10 ms delay between retries (can be adjusted)
    }

    // If we exit the loop without finding the IMU, handle the timeout
    debug_print("Error: IMU search exceeded maximum attempts\r\n");
}

// Function to configure the MPU-6050 for the fastest mode with the low-pass filter
void IMU_Config_Fast_Mode(void) {
    uint8_t data[2];

    // Step 1: Wake up the MPU-6050 (write 0x00 to PWR_MGMT_1 register)
    data[0] = 0x00;  // Wake up the IMU by writing 0 to PWR_MGMT_1 register
    if (I2C_Write_DMA(IMU_I2C_ADDRESS, PWR_MGMT_1_REG, data, 1) != HAL_OK) {
        debug_print("Error: Failed to wake up IMU\r\n");
        return;  // Exit if I2C write failed
    }

    // Step 2: Configure the low-pass filter (DLPF)
    // Set the DLPF to the fastest mode (Low-pass filter disabled)
    data[0] = 0x00;  // DLPF_CFG = 0 (highest data rate)
    if (I2C_Write_DMA(IMU_I2C_ADDRESS, CONFIG_REG, data, 1) != HAL_OK) {
        debug_print("Error: Failed to set DLPF\r\n");
        return;
    }

    // Step 3: Configure accelerometer range to ±16g (fastest mode)
    data[0] = 0x18;  // AFS_SEL = 0x03 (±16g range)
    if (I2C_Write_DMA(IMU_I2C_ADDRESS, ACCEL_CONFIG_REG, data, 1) != HAL_OK) {
        debug_print("Error: Failed to set accelerometer range\r\n");
        return;
    }

    // Step 4: Configure gyroscope range to ±2000°/s (fastest mode)
    data[0] = 0x18;  // FS_SEL = 0x03 (±2000°/s range)
    if (I2C_Write_DMA(IMU_I2C_ADDRESS, GYRO_CONFIG_REG, data, 1) != HAL_OK) {
        debug_print("Error: Failed to set gyroscope range\r\n");
        return;
    }

    // Step 5: Set sampling rate to the maximum (SMPLRT_DIV = 0)
    data[0] = 0x00;  // SMPLRT_DIV = 0 for max sampling rate (1kHz)
    if (I2C_Write_DMA(IMU_I2C_ADDRESS, SMPLRT_DIV_REG, data, 1) != HAL_OK) {
        debug_print("Error: Failed to set sampling rate\r\n");
        return;
    }

    // Print confirmation
    debug_print("MPU-6050 configured for fastest mode with low-pass filter disabled\r\n");
}

// Function to read accelerometer and gyroscope data
void IMU_Read_Accel_Gyro(void) {
    char str[64];

    // Start I2C read with DMA for accelerometer and gyroscope data (14 bytes)
    I2C_Read_DMA(IMU_I2C_ADDRESS, 0x3B, imu_data, 14);

    // Wait for DMA read to complete with timeout logic
    uint32_t wait_start_time = get_millis();
    uint32_t timeout_duration = 1000;  // Timeout duration for reading (in ms)

    // Wait for DMA transfer to complete or timeout
    while (!dma_read_complete) {
        if (get_millis() - wait_start_time > timeout_duration) {
            debug_print("Error: DMA read timeout\r\n");
            return;  // Exit if the read operation times out
        }
    }

    // Process accelerometer data
    int16_t accel_x = (int16_t)((imu_data[0] << 8) | imu_data[1]);
    int16_t accel_y = (int16_t)((imu_data[2] << 8) | imu_data[3]);
    int16_t accel_z = (int16_t)((imu_data[4] << 8) | imu_data[5]);

    // Process gyroscope data
    int16_t gyro_x = (int16_t)((imu_data[8] << 8) | imu_data[9]);
    int16_t gyro_y = (int16_t)((imu_data[10] << 8) | imu_data[11]);
    int16_t gyro_z = (int16_t)((imu_data[12] << 8) | imu_data[13]);
/*
    // Print accelerometer data with padding to clear leftovers
    sprintf(str, "\033[1;1HAccel X: %d, Y: %d, Z: %d       ", accel_x, accel_y, accel_z);
    debug_print(str);

    // Print gyroscope data with padding to clear leftovers
    sprintf(str, "\033[2;1HGyro X: %d, Y: %d, Z: %d       ", gyro_x, gyro_y, gyro_z);
    debug_print(str);*/

    // Print accelerometer data with padding to clear leftovers
    sprintf(str, "AccelX:%d, Y:%d, Z:%d", accel_x, accel_y, accel_z);
    debug_print(str);

    // Print gyroscope data with padding to clear leftovers
    sprintf(str, "GyroX:%d, Y:%d, Z:%d", gyro_x, gyro_y, gyro_z);
    debug_print(str);
}

void debug_print(char* str){
	USART1_Transmit_DMA(str, strlen(str));
}

void USART1_Transmit_DMA(char *data, uint16_t size) {
    if (size == 0) return;  // Ignore empty data (nothing to transmit)

    __disable_irq();  // Prevent race conditions

    // Calculate available free space in the buffer
    uint16_t freeSpace = (txReadIndex > txWriteIndex) ?
                         (txReadIndex - txWriteIndex - 1) :  					// Buffer space available when read index is ahead of write index
                         ((UART_TX_BUFFER_SIZE - txWriteIndex) + txReadIndex - 1);  	// Buffer space available when write index has wrapped

    // Limit the size to the available free space
    if (size > freeSpace) {
        size = freeSpace;  // Drop excess characters
    }

    // Copy only the allowed size into the buffer
    for (uint16_t i = 0; i < size; i++) {
        uart1_txBuffer[txWriteIndex] = data[i];  								// Copy data into the buffer
        txWriteIndex = (txWriteIndex + 1) % UART_TX_BUFFER_SIZE;  					// Move write pointer circularly
    }

    // Start transmission if not already in progress
    if (!txBusy && size > 0) {
        txBusy = 1;  // Mark that transmission is in progress

        // Calculate the chunk size that needs to be sent (based on available data)
        uint16_t chunkSize = (txWriteIndex >= txReadIndex) ?
                             (txWriteIndex - txReadIndex) :  					// If data is in one chunk
                             (UART_TX_BUFFER_SIZE - txReadIndex); 					// If data is split across buffer boundary

        HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&uart1_txBuffer[txReadIndex], chunkSize);  	// Start DMA transmission
    }

    __enable_irq();  															// Re-enable interrupts
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    //if (huart->Instance == USART1) {  // Check if the callback is for USART1
        // Update txReadIndex to point to the next byte to be transmitted
        txReadIndex = (txReadIndex + huart->TxXferSize) % UART_TX_BUFFER_SIZE;

        // If there's more data to transmit, continue the transmission
        if (txReadIndex != txWriteIndex) {
            // Calculate the chunk size (remaining data to send)
            uint16_t chunkSize = (txWriteIndex > txReadIndex) ?
                                 (txWriteIndex - txReadIndex) :  // Data is in one chunk
                                 (UART_TX_BUFFER_SIZE - txReadIndex);  // Data is split across the buffer

            // Continue transmission using DMA
            HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&uart1_txBuffer[txReadIndex], chunkSize);
        } else {
            // No more data to transmit, transmission is complete
            txBusy = 0;  // Mark the transmission as not busy
        }
    //}
}

// Function to write data to the IMU register using I2C with DMA
HAL_StatusTypeDef I2C_Write_DMA(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t size) {
    // Send the register address followed by the data
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write_DMA(&hi2c1, devAddr, regAddr, I2C_MEMADD_SIZE_8BIT, data, size);
    return status;
}

// Function to read data from the IMU register using I2C with DMA
HAL_StatusTypeDef I2C_Read_DMA(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t size) {
    // Send the register address and then read the data
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read_DMA(&hi2c1, devAddr, regAddr, I2C_MEMADD_SIZE_8BIT, data, size);
    return status;
}


// DMA complete callback
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	  UNUSED(hi2c);
	// Set the flag to indicate that the DMA read operation is complete
    dma_read_complete = 1;
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C1) {
        // Handle transmit complete (DMA transfer completed)
    }
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C1) {
        // Handle receive complete (DMA transfer completed)
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C1) {
        // Handle errors (e.g., NACK, timeouts, etc.)
    }
}

// Function to get the current time in milliseconds
uint32_t get_millis(void) {
    return millis;
}

// Function to create a delay in milliseconds (blocking)
void delay(uint32_t ms) {
    uint32_t start = get_millis();
    while (get_millis() - start < ms) {
        // Wait for the desired amount of time to pass
    }
}

// Function to check for timeout (non-blocking)
int timeout(uint32_t start_time, uint32_t timeout_period) {
    uint32_t current_time = get_millis();

    // Handle overflow using unsigned comparison
    if ((current_time - start_time) >= timeout_period) {
        return 1;  // Timeout occurred
    }
    return 0;  // No timeout yet
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
