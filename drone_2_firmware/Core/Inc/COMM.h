/*
 * COMM.h
 *
 *  Created on: Apr 2, 2026
 *      Author: balin
 */

#ifndef INC_COMM_H_
#define INC_COMM_H_

#include "main.h"

#define UART_TX_BUFFER_SIZE 		1024
#define UART_RX_BUFFER_SIZE 		32
#define I2C_TX_BUFFER_SIZE 			32
#define I2C_RX_BUFFER_SIZE 			32
#define I2C_QUEUE_LEN 				8
#define SENSOR_BUF_SIZE 			128

//Optical sensor
#define BURST_SIZE 12

typedef enum {
    SPI_STATE_IDLE = 0,
    SPI_STATE_BUSY = 1,
    SPI_STATE_DONE = 2,
    SPI_STATE_ERROR = 3
} SPI_State_t;

typedef enum {
    SENSOR_TYPE_IMU,
	SENSOR_TYPE_TOF,
} sensor_type_t;


typedef enum {
    I2C_IDLE = 0,
	I2C_BUSY,
} i2c_state_t;

typedef enum {
    SENSOR_IDLE = 0,
	SENSOR_BUSY,
} Sensor_State_t;

typedef enum
{
    I2C_DEVICE_NONE = 0,
    I2C_DEVICE_IMU,
    I2C_DEVICE_TOF

} i2c_device_t;

typedef enum
{
	I2C_REQ_READ = 0,
	I2C_REQ_WRITE,

} i2c_direction_t;

typedef enum {
	NOT_TOF		= 0,
    TOF_STATUS  	= 1,
	TOF_DISTANCE 	= 2,  	// Bit 0 of Register 0x3A
	TOF_CLR_IT		= 3,
} tof_raw_data_type_t;

typedef struct {
	i2c_direction_t direction;
	sensor_type_t type;
	tof_raw_data_type_t tof_type;
    uint8_t devAddr;
    uint16_t regAddr;
    uint8_t *data;
    uint16_t size;
    uint8_t isRead;
    uint16_t regSize;
} i2c_req_t;

typedef struct
{
	i2c_req_t buf[SENSOR_BUF_SIZE];
    volatile uint8_t head;
    volatile uint8_t tail;

} i2c_req_rb_t;

typedef enum {
    IMU_DATA_NOT_READY = 0x00,
    IMU_DATA_READY     = 0x01  	// Bit 0 of Register 0x3A
} IMU_Data_rdy;

typedef struct {
	IMU_Data_rdy status;       	// Register 0x3A (contains Data Ready bit)
    uint8_t imu_data[14]; 		// Registers 0x3B to 0x48
}imu_raw_data_t;
/*
typedef struct
{
	uint8_t tof_raw_buf[32];
} tof_raw_data_t;*/


typedef struct {
    sensor_type_t type;
    tof_raw_data_type_t tof_type;
    uint32_t timestamp;
    union {
    	imu_raw_data_t imu_raw;
    	uint8_t tof_raw_buf[2];
    } data;
} sensor_event_t;

typedef enum {
    OPT_IDLE = 0,
    OPT_TX,
    OPT_RX
} OpticalState_t;

typedef struct {
    sensor_event_t buf[SENSOR_BUF_SIZE];
    volatile uint8_t head;
    volatile uint8_t tail;
} sensor_rb_t;


extern I2C_HandleTypeDef hi2c1;
extern DMA_HandleTypeDef hdma_i2c1_tx;
extern DMA_HandleTypeDef hdma_i2c1_rx;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern SPI_HandleTypeDef hspi2;

extern volatile uint16_t txWriteIndex;
extern volatile uint16_t txReadIndex;
extern volatile uint8_t txBusy;

extern i2c_state_t i2c_dma_state;
extern volatile i2c_device_t active_i2c_device;
extern i2c_req_t TOF;
extern i2c_req_t IMU;
extern uint8_t tof_status;

extern SPI_State_t spi_state;
extern uint8_t spi_buff[BURST_SIZE];

void comm_init();
void MX_SPI2_Init_AFTER_OPTIC(void);
HAL_StatusTypeDef I2C_Write_DMA(uint8_t devAddr, uint16_t regAddr, uint8_t *data, uint16_t size);
HAL_StatusTypeDef I2C_Read_DMA(uint8_t devAddr, uint16_t regAddr, uint8_t *data, uint16_t size);
void USART1_Transmit_DMA(char *data, uint16_t size);
void debug_print(char* str);
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);
void I2C_Task(void);
void IMU_Task(void);
void TOF_Task_Status(void);
void TOF_Task_Distance(void);
void TOF_Task_Clear(void);
uint8_t SENSOR_Process(void);
#endif /* INC_COMM_H_ */
