/*
 * COMM.c
 *
 *  Created on: Apr 2, 2026
 *      Author: balin
 */
#include "main.h"

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_tx;
DMA_HandleTypeDef hdma_i2c1_rx;
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;
SPI_HandleTypeDef hspi2;

char uart1_txBuffer[UART_TX_BUFFER_SIZE];  // Transmit buffer
char uart1_rxBuffer[UART_RX_BUFFER_SIZE];  // Receive buffer (if needed)
volatile uint16_t txWriteIndex = 0;
volatile uint16_t txReadIndex = 0;
volatile uint8_t txBusy = 0;

i2c_state_t i2c_state = I2C_IDLE;

Sensor_State_t imu_task_state = SENSOR_IDLE;
Sensor_State_t tof_task_state = SENSOR_IDLE;
static sensor_rb_t sensor_sample_rb = {0};
static i2c_req_rb_t sensor_request_rb  = {0};
static imu_raw_data_t imu_raw_tmp  = {0};
uint8_t tof_status = 0;
static uint8_t tof_raw_tmp[2]  = {0};
i2c_req_t current_req  = {0};

SPI_State_t spi_state;
uint8_t spi_buff[BURST_SIZE] = {0};

void comm_init();
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_DMA_Init(void);

void comm_init()
{
	MX_DMA_Init();
	MX_USART1_UART_Init();
	MX_I2C1_Init();
	MX_SPI2_Init();
	MX_SPI2_Init_AFTER_OPTIC();
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
  huart1.Init.BaudRate = 921600;
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
/* USER CODE BEGIN SPI2_Init 2 */
static void MX_SPI2_Init(void)
{

	/* USER CODE END SPI2_Init 1 */
	/* SPI2 parameter configuration*/
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
	hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
	hspi2.Init.NSS = SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 7;
	hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
	if (HAL_SPI_Init(&hspi2) != HAL_OK)
	{
	Error_Handler();
	}

}

/* USER CODE BEGIN SPI2_Init 2 */
void MX_SPI2_Init_AFTER_OPTIC(void)
{

	/* USER CODE END SPI2_Init 1 */
	/* SPI2 parameter configuration*/
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
	hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
	hspi2.Init.NSS = SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 7;
	hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
	if (HAL_SPI_Init(&hspi2) != HAL_OK)
	{
	Error_Handler();
	}

}
/* USER CODE END SPI2_Init 2 */

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

static inline int i2c_req_push(i2c_req_t *req)
{

	unsigned int next = (sensor_request_rb.head + 1) & (SENSOR_BUF_SIZE - 1);

	if (next == sensor_request_rb.tail)
	{
		return 0; // Buffer Full
	}

	sensor_request_rb.buf[sensor_request_rb.head] = *req;

	// If not using atomics/volatile, a compiler barrier here is highly recommended:
	// __atomic_signal_fence(__ATOMIC_RELEASE);

	sensor_request_rb.head = next;
	return 1;
}

static inline int i2c_req_pop(i2c_req_t *req)
{
	if (sensor_request_rb.head == sensor_request_rb.tail)
	    {
	        return 0; // Buffer Empty
	    }

	*req = sensor_request_rb.buf[sensor_request_rb.tail];

	// __atomic_signal_fence(__ATOMIC_RELEASE);

	sensor_request_rb.tail = (sensor_request_rb.tail + 1) & (SENSOR_BUF_SIZE - 1);
	return 1;
}

static inline void sensor_rb_push(sensor_event_t *e)
{
    uint8_t next = (sensor_sample_rb.head + 1) % SENSOR_BUF_SIZE;

    if (next == sensor_sample_rb.tail)
    {
        return; // overflow drop
    }
    sensor_sample_rb.buf[sensor_sample_rb.head] = *e;
    sensor_sample_rb.head = next;
}

static inline int sensor_rb_pop(sensor_event_t *e)
{
    if (sensor_sample_rb.head == sensor_sample_rb.tail)
    {
    	return 0;
    }
    *e = sensor_sample_rb.buf[sensor_sample_rb.tail];
    sensor_sample_rb.tail = (sensor_sample_rb.tail + 1) % SENSOR_BUF_SIZE;

    return 1;
}

void IMU_Task(void)
{
    if (imu_task_state == SENSOR_BUSY)
    {
    	return;
   	}

    static i2c_req_t req = {0};
    req.direction = I2C_REQ_READ;
    req.type = SENSOR_TYPE_IMU;
    req.tof_type = NOT_TOF;
    req.devAddr = IMU_I2C_ADDRESS;

    req.regAddr = IMU_DATA_BURST;
    req.regSize = I2C_MEMADD_SIZE_8BIT;

    req.data = imu_raw_tmp.imu_data;
    req.size = 14;

    if (i2c_req_push(&req))
    {
    	imu_task_state = SENSOR_BUSY;
    }
}

void TOF_Task_Status(void)
{
    if (tof_task_state == SENSOR_BUSY)
        return;

    static i2c_req_t req;

    req.direction = I2C_REQ_READ;
    req.type = SENSOR_TYPE_TOF;
    req.tof_type = TOF_STATUS;
    req.devAddr = TOF_I2C_ADDRESS;

    // burst read of result block
    req.regAddr = RESULT_INTERRUPT_STATUS;
    req.regSize = I2C_MEMADD_SIZE_16BIT;

    req.data = &tof_status;
    req.size = 1;

    tof_task_state = SENSOR_BUSY;
    i2c_req_push(&req);
}

void TOF_Task_Clear(void)
{
    if (tof_task_state == SENSOR_BUSY)
        return;

    static i2c_req_t req;
    static uint8_t clear = 0x01;

    req.direction = I2C_REQ_WRITE;
    req.type = SENSOR_TYPE_TOF;
    req.tof_type = TOF_CLR_IT;
    req.devAddr = TOF_I2C_ADDRESS;

    // burst read of result block
    req.regAddr = TOF_SYSTEM_INTERRUPT_CLEAR;
    req.regSize = I2C_MEMADD_SIZE_16BIT;
    req.data = &clear;
    req.size = 1;

    tof_task_state = SENSOR_BUSY;
    i2c_req_push(&req);
}

void TOF_Task_Distance()
{
    if (tof_task_state == SENSOR_BUSY)
        return;

    static i2c_req_t req;

    req.direction = I2C_REQ_READ;
    req.type = SENSOR_TYPE_TOF;
    req.tof_type = TOF_DISTANCE;
    req.devAddr = TOF_I2C_ADDRESS;

    // burst read of result block
    req.regAddr = VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0;
    req.regSize = I2C_MEMADD_SIZE_16BIT;
    req.data = tof_raw_tmp;
    req.size = 2;

    tof_task_state = SENSOR_BUSY;
    i2c_req_push(&req);
}

void I2C_Task(void)
{
    if (i2c_state == I2C_BUSY)
        return;

    if (!i2c_req_pop(&current_req))
        return;

    i2c_state = I2C_BUSY;

    HAL_StatusTypeDef st;

    switch(current_req.direction)
    {
        case I2C_REQ_READ:

            st = HAL_I2C_Mem_Read_DMA(&hi2c1, current_req.devAddr, current_req.regAddr, current_req.regSize, current_req.data, current_req.size);
            break;

        case I2C_REQ_WRITE:

            st = HAL_I2C_Mem_Write_DMA(&hi2c1, current_req.devAddr, current_req.regAddr, current_req.regSize, current_req.data, current_req.size);
            break;

        default:
            st = HAL_ERROR;
            break;
    }

    if (st != HAL_OK)
    {
    	i2c_state = I2C_IDLE;
    }
}

uint8_t SENSOR_Process(void)
{
    uint8_t ret_val = 0;
    sensor_event_t e;

    while (sensor_rb_pop(&e))
    {
        switch (e.type)
        {
            case SENSOR_TYPE_IMU:
                IMU_Parse_Data(e.data.imu_raw, e.timestamp);
                ret_val = 1;
                break;

            case SENSOR_TYPE_TOF:
            	switch(e.tof_type)
            	{
            		case TOF_STATUS:
                    	TOF_Parse_Status(e.data.tof_raw_buf, e.timestamp);
            			break;
            		case TOF_DISTANCE:
            			TOF_Parse_Distance(e.data.tof_raw_buf, e.timestamp);
            			break;
            		case TOF_CLR_IT:
            			return 0;
            			break;
            		case NOT_TOF:
            			return 0;
            			break;

                ret_val = 1;
            	}
                break;

            default:
                break;
        }
    }

    return ret_val;
}

// DMA complete callback
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {

	    UNUSED(hi2c);
	    UNUSED(hi2c);

	    i2c_state = I2C_IDLE;

	    sensor_event_t e;

	    // capture timestamp as early as possible
	    e.timestamp = DWT->CYCCNT;

	    switch (current_req.devAddr)
	    {
	        case IMU_I2C_ADDRESS:
	        {
	            e.type = SENSOR_TYPE_IMU;

	            // copy status byte if it is part of burst
	            e.data.imu_raw.status = imu_raw_tmp.status;

	            // copy raw burst (14 bytes)
	            memcpy(e.data.imu_raw.imu_data, imu_raw_tmp.imu_data, 14);

	            // optional: timestamp from sensor side if you already compute it
	            sensor_rb_push(&e);
	            imu_task_state = SENSOR_IDLE;
	            break;
	        }
	        case TOF_I2C_ADDRESS:
	        {
	            e.timestamp = DWT->CYCCNT;
	            e.type = SENSOR_TYPE_TOF;
	            e.tof_type = current_req.tof_type;

	            memcpy(&e.data.tof_raw_buf, &tof_raw_tmp, sizeof(e.data.tof_raw_buf));

	            sensor_rb_push(&e);

	            tof_task_state = SENSOR_IDLE;
	            break;
	        }

	        default:
	            break;
	    }
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c) {
	UNUSED(hi2c);
	if (hi2c != &hi2c1)
		return;

	i2c_state = I2C_IDLE;

    switch (current_req.devAddr)
    {
        case IMU_I2C_ADDRESS:
        {
            imu_task_state = SENSOR_IDLE;
            break;
        }
        case TOF_I2C_ADDRESS:
        {
            tof_task_state = SENSOR_IDLE;
            break;
        }

        default:
            break;
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C1) {
        // Handle errors (e.g., NACK, timeouts, etc.)
    }
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

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
    UNUSED(hspi);
    if (hspi->Instance != SPI2)
        return;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    opt_read_state = 1;
    spi_state = SPI_STATE_DONE;
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
    UNUSED(hspi);
    if (hspi->Instance != SPI2)
        return;

    //HAL_SPI_Receive_IT(&hspi2, spi_buff, 7);
    spi_state = SPI_STATE_BUSY;
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	UNUSED(hspi);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    opt_read_state = 1;
    spi_state = SPI_STATE_DONE;
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance != SPI2)
        return;

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

    spi_state = SPI_STATE_IDLE;
}
