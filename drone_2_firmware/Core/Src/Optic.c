/*
 * Optic.c
 *
 *  Created on: 2026. máj. 7.
 *      Author: balin
 */

#include "main.h"

#define FLOW_SCALE 0.001

PMW3901_Data_t optical_data = {0};
uint32_t opt_now = 0;
uint32_t opt_deltatime = 0;
uint32_t opt_deltatime_last = 0;
float opt_deltatime_f = 0.0f;
uint8_t opt_read_state = 0;
uint32_t opt_last_cycle = 0;
float vx_flow_world = 0;
float vy_flow_world = 0;

void PMW3901_Write(uint8_t reg, uint8_t value)
{
    uint8_t tx[2];

    tx[0] = reg | 0x80; // write bit
    tx[1] = value;

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

    HAL_SPI_Transmit(&hspi2, tx, 2, 100);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

    delay_us(50);
}

uint8_t PMW3901_Read(uint8_t reg)
{
    uint8_t tx;
    uint8_t rx;

    tx = reg & 0x7F;

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

    HAL_SPI_Transmit(&hspi2, &tx, 1, 100);

    delay_us(35);

    HAL_SPI_Receive(&hspi2, &rx, 1, 100);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

    delay_us(1);

    return rx;
}

// ============================================================================
// Full initialization
// ============================================================================

uint8_t PMW3901_Init(void)
{
    // Reset SPI state machine
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    HAL_Delay(1);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_Delay(1);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    HAL_Delay(1);

    // Power-on reset
    PMW3901_Write(0x3A, 0x5A);
    HAL_Delay(5);

    // Verify chip
    uint8_t chipId    = PMW3901_Read(0x00);
    uint8_t chipIdInv = PMW3901_Read(0x5F);

    if ((chipId != 0x49) || (chipIdInv != 0xB6))
    {
    	sprintf(str,"PMW3901 ID ERROR: %02X %02X\r\n", chipId, chipIdInv);
        debug_print(str);
        return 0;
    }

    // Clear startup motion state
    PMW3901_Read(0x02);
    PMW3901_Read(0x03);
    PMW3901_Read(0x04);
    PMW3901_Read(0x05);
    PMW3901_Read(0x06);

    HAL_Delay(1);

    // Load Bitcraze/PixArt initialization table
    PMW3901_SecretSauce();

    debug_print("PMW3901 OK\r\n");

    return 1;
}

void PMW3901_SecretSauce(void)
{
	 PMW3901_Write(0x7F, 0x00);
	    PMW3901_Write(0x61, 0xAD);
	    PMW3901_Write(0x7F, 0x03);
	    PMW3901_Write(0x40, 0x00);
	    PMW3901_Write(0x7F, 0x05);
	    PMW3901_Write(0x41, 0xB3);
	    PMW3901_Write(0x43, 0xF1);
	    PMW3901_Write(0x45, 0x14);
	    PMW3901_Write(0x5B, 0x32);
	    PMW3901_Write(0x5F, 0x34);
	    PMW3901_Write(0x7B, 0x08);

	    PMW3901_Write(0x7F, 0x06);
	    PMW3901_Write(0x44, 0x1B);
	    PMW3901_Write(0x40, 0xBF);
	    PMW3901_Write(0x4E, 0x3F);

	    PMW3901_Write(0x7F, 0x08);
	    PMW3901_Write(0x65, 0x20);
	    PMW3901_Write(0x6A, 0x18);

	    PMW3901_Write(0x7F, 0x09);
	    PMW3901_Write(0x4F, 0xAF);
	    PMW3901_Write(0x5F, 0x40);
	    PMW3901_Write(0x48, 0x80);
	    PMW3901_Write(0x49, 0x80);
	    PMW3901_Write(0x57, 0x77);
	    PMW3901_Write(0x60, 0x78);
	    PMW3901_Write(0x61, 0x78);
	    PMW3901_Write(0x62, 0x08);
	    PMW3901_Write(0x63, 0x50);

	    PMW3901_Write(0x7F, 0x0A);
	    PMW3901_Write(0x45, 0x60);

	    PMW3901_Write(0x7F, 0x00);
	    PMW3901_Write(0x4D, 0x11);
	    PMW3901_Write(0x55, 0x80);
	    PMW3901_Write(0x74, 0x1F);
	    PMW3901_Write(0x75, 0x1F);
	    PMW3901_Write(0x4A, 0x78);
	    PMW3901_Write(0x4B, 0x78);
	    PMW3901_Write(0x44, 0x08);
	    PMW3901_Write(0x45, 0x50);
	    PMW3901_Write(0x64, 0xFF);
	    PMW3901_Write(0x65, 0x1F);

	    PMW3901_Write(0x7F, 0x14);
	    PMW3901_Write(0x65, 0x60);
	    PMW3901_Write(0x66, 0x08);
	    PMW3901_Write(0x63, 0x78);

	    PMW3901_Write(0x7F, 0x15);
	    PMW3901_Write(0x48, 0x58);

	    PMW3901_Write(0x7F, 0x07);
	    PMW3901_Write(0x41, 0x0D);
	    PMW3901_Write(0x43, 0x14);
	    PMW3901_Write(0x4B, 0x0E);
	    PMW3901_Write(0x45, 0x0F);
	    PMW3901_Write(0x44, 0x42);
	    PMW3901_Write(0x4C, 0x80);

	    PMW3901_Write(0x7F, 0x10);
	    PMW3901_Write(0x5B, 0x02);

	    PMW3901_Write(0x7F, 0x07);
	    PMW3901_Write(0x40, 0x41);
	    PMW3901_Write(0x70, 0x00);

	    HAL_Delay(100);

	    PMW3901_Write(0x32, 0x44);

	    PMW3901_Write(0x7F, 0x07);
	    PMW3901_Write(0x40, 0x40);

	    PMW3901_Write(0x7F, 0x06);
	    PMW3901_Write(0x62, 0xF0);
	    PMW3901_Write(0x63, 0x00);

	    PMW3901_Write(0x7F, 0x0D);
	    PMW3901_Write(0x48, 0xC0);
	    PMW3901_Write(0x6F, 0xD5);

	    PMW3901_Write(0x7F, 0x00);
	    PMW3901_Write(0x5B, 0xA0);
	    PMW3901_Write(0x4E, 0xA8);
	    PMW3901_Write(0x5A, 0x50);
	    PMW3901_Write(0x40, 0x80);
}

void optical_request_motion(void)
{
	/*uint8_t spi_tx[1];
    if(!opt_read_state)
    {
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
        uint8_t tx = PMW3901_REG_MOTION_BURST;

        //HAL_SPI_Transmit_IT(&hspi2, &tx, 1);8
        HAL_SPI_TransmitReceive_IT(&hspi2, &tx, spi_buff, 13);
    }*/
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

	uint8_t reg = PMW3901_REG_MOTION_BURST;
	HAL_SPI_Transmit(&hspi2, &reg, 1, 10);

	HAL_SPI_Receive(&hspi2, spi_buff, 7, 10);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

	optical_parse(spi_buff);
}

uint8_t PMW3901_Read_reg(uint8_t reg)
{
    uint8_t dummy = 0x00;
    uint8_t rx;

    reg &= 0x7F;

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

    delay_us(50);

    HAL_SPI_Transmit(&hspi2, &reg, 1, HAL_MAX_DELAY);

    delay_us(50);

    HAL_SPI_TransmitReceive(&hspi2, &dummy, &rx, 1, HAL_MAX_DELAY);

    delay_us(100);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

    return rx;
}

void optical_parse(uint8_t* raw)
{
	/*if(!opt_read_state)
	{
		return;
	}*/
	optical_data.motion = raw[0];
	optical_data.squal   = raw[6];

	if(optical_data.squal < 10)
	{
		//optical_data.deltaX = 0;
		//optical_data.deltaY = 0;
		//sprintf(str,"squal: %i\r\n", optical_data.squal);
		//debug_print(str);
		return;
	}

	optical_data.deltaX = (int16_t)((raw[3] << 8) | raw[2]);
	//optical_data.deltaX = - optical_data.deltaX;
	optical_data.deltaY = (int16_t)((raw[5] << 8) | raw[4]);

	uint32_t now = get_us();
	opt_deltatime = now - opt_deltatime_last;
	opt_deltatime_last = now;
	opt_deltatime_f = (float) opt_deltatime * INV_CPU_FREQ;
	//sprintf(str,"squal: %i\r\n", optical_data.squal);
	//debug_print(str);
	//sprintf(str,"opt x: %i, opt y: %i\r\n", optical_data.deltaX, optical_data.deltaY);
	//debug_print(str);
	body_to_world(optical_data.deltaX, optical_data.deltaY, euler_flt.yaw * DEG2RAD, opt_deltatime_f, vertical_state.z, 0.0005f, &vx_flow_world, &vy_flow_world);
	//sprintf(str,"opt x: %f, opt y: %f\r\n", vx_flow_world, vy_flow_world);
	//debug_print(str);
	opt_read_state = 0;
}

void body_to_world(int dx, int dy,  float yaw,  float dt,  float height,   float scale,   float *vx_world,  float *vy_world)
{
    float c = cosf(yaw);
    float s = sinf(yaw);

    float vx_body = ((float)dx / dt) * scale * height;
    float vy_body = ((float)dy / dt) * scale * height;

    *vx_world = c * vx_body - s * vy_body;
    *vy_world = s * vx_body + c * vy_body;
}


