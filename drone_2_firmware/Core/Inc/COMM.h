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

extern DMA_HandleTypeDef hdma_i2c1_tx;
extern DMA_HandleTypeDef hdma_i2c1_rx;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim6;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern UART_HandleTypeDef huart1;

extern volatile uint16_t txWriteIndex;
extern volatile uint16_t txReadIndex;
extern volatile uint8_t txBusy;
extern char str_tmp[UART_TX_BUFFER_SIZE];

extern uint8_t i2c1_txBuffer[I2C_TX_BUFFER_SIZE];
extern uint8_t i2c1_rxBuffer[I2C_RX_BUFFER_SIZE];
extern uint8_t dma_read_complete;
extern uint8_t dma_write_complete;

HAL_StatusTypeDef I2C_Write_DMA(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t size);
HAL_StatusTypeDef I2C_Read_DMA(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t size);
void USART1_Transmit_DMA(char *data, uint16_t size);
void debug_print(char* str);
void print_float_quat(quat_q12_t , uint8_t *);
void print_float_vec3(vec3_q12_t , uint8_t *);
#endif /* INC_COMM_H_ */
