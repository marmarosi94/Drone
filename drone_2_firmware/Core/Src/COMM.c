/*
 * COMM.c
 *
 *  Created on: Apr 2, 2026
 *      Author: balin
 */
#include "COMM.h"

char uart1_txBuffer[UART_TX_BUFFER_SIZE];  // Transmit buffer
char uart1_rxBuffer[UART_RX_BUFFER_SIZE];  // Receive buffer (if needed)

volatile uint16_t txWriteIndex = 0;
volatile uint16_t txReadIndex = 0;
volatile uint8_t txBusy = 0;
char str_tmp[UART_TX_BUFFER_SIZE] = {0};

uint8_t i2c1_txBuffer[I2C_TX_BUFFER_SIZE];
uint8_t i2c1_rxBuffer[I2C_RX_BUFFER_SIZE];
uint8_t dma_read_complete = 0;
uint8_t dma_write_complete = 0;


// Function to write data to the IMU register using I2C with DMA
HAL_StatusTypeDef I2C_Write_DMA(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t size) {
    // Send the register address followed by the data
	dma_write_complete = 0;
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write_DMA(&hi2c1, devAddr, regAddr, I2C_MEMADD_SIZE_8BIT, data, size);
    return status;
}

// Function to read data from the IMU register using I2C with DMA
HAL_StatusTypeDef I2C_Read_DMA(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t size) {
    // Send the register address and then read the data
	dma_read_complete = 0;
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read_DMA(&hi2c1, devAddr, regAddr, I2C_MEMADD_SIZE_8BIT, data, size);
    return status;
}


// DMA complete callback
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	  UNUSED(hi2c);
	// Set the flag to indicate that the DMA read operation is complete
    dma_read_complete = 1;
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c) {
	  UNUSED(hi2c);
	// Set the flag to indicate that the DMA read operation is complete
	  dma_write_complete = 1;
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C1) {
    	dma_write_complete = 1;
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

void print_float_quat(quat_q16_t obj, uint8_t * str_float){
	// Helper to get printable parts
	int32_t x_int = obj.x / 4096;
	int32_t y_int = obj.y / 4096;
	int32_t z_int = obj.z / 4096;
	int32_t w_int = obj.w / 4096;
	// Scale remainder to 1000 for 3 decimal places: (rem * 1000) / 4096
	int32_t x_frac = abs((int32_t)obj.x % 4096) * 1000 / 4096;
	int32_t y_frac = abs((int32_t)obj.y % 4096) * 1000 / 4096;
	int32_t z_frac = abs((int32_t)obj.z % 4096) * 1000 / 4096;
	int32_t w_frac = abs((int32_t)obj.w % 4096) * 1000 / 4096;

    sprintf(str_float, "$%i.%i,%i.%i,%i.%i,%i.%i\r\n", x_int, x_frac, y_int,y_frac, z_int, z_frac, w_int, w_frac);
    debug_print(str_float);  // Send gyroscope data to PC
}

void print_float_vec3(vec3_q16_t obj, uint8_t * str_float){
	// Helper to get printable parts
	int32_t x_int = obj.x / 4096;
	int32_t y_int = obj.y / 4096;
	int32_t z_int = obj.z / 4096;
	// Scale remainder to 1000 for 3 decimal places: (rem * 1000) / 4096
	int32_t x_frac = abs((int32_t)obj.x % 4096) * 1000 / 4096;
	int32_t y_frac = abs((int32_t)obj.y % 4096) * 1000 / 4096;
	int32_t z_frac = abs((int32_t)obj.z % 4096) * 1000 / 4096;

    sprintf(str_float, "%i.%i,%i.%i,%i.%i\r\n", x_int, x_frac, y_int,y_frac, z_int, z_frac);
    debug_print(str_float);  // Send gyroscope data to PC
}
