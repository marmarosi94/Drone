/*
 * COMM.h
 *
 *  Created on: Apr 2, 2026
 *      Author: balin
 */

#ifndef INC_COMM_H_
#define INC_COMM_H_

#include "main.h"

#define UART_TX_BUFFER_SIZE 1024
#define UART_RX_BUFFER_SIZE 32
#define I2C_TX_BUFFER_SIZE 32
#define I2C_RX_BUFFER_SIZE 32

typedef enum {
    I2C_IDLE = 0,
    I2C_BUSY = 1,
    I2C_COMPLETE = 2
} I2C_State_t;

extern volatile uint16_t txWriteIndex;
extern volatile uint16_t txReadIndex;
extern volatile uint8_t txBusy;
extern char str_tmp[UART_TX_BUFFER_SIZE];

extern uint8_t i2c1_txBuffer[I2C_TX_BUFFER_SIZE];
extern uint8_t i2c1_rxBuffer[I2C_RX_BUFFER_SIZE];
extern I2C_State_t dma_read_complete;
extern I2C_State_t dma_write_complete;

HAL_StatusTypeDef I2C_Write_DMA(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t size);
HAL_StatusTypeDef I2C_Read_DMA(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t size);
void USART1_Transmit_DMA(char *data, uint16_t size);
void debug_print(char* str);
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);
#endif /* INC_COMM_H_ */
