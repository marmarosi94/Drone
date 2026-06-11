/**
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#include "vl53l1_platform.h"
#include <string.h>
#include <time.h>
#include <math.h>
#include "main.h"

int8_t VL53L1_WriteMulti( uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count) {
    uint8_t buf[260];

    if (count + 2 > sizeof(buf))
        return -1;

    buf[0] = index >> 8;
    buf[1] = index & 0xFF;

    memcpy(&buf[2], pdata, count);

    if (HAL_I2C_Master_Transmit(&hi2c1, dev, buf, count + 2, 100) != HAL_OK)
        return -1;

    return 0;
}

int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count){
    uint8_t reg[2];

    reg[0] = index >> 8;
    reg[1] = index & 0xFF;

    if (HAL_I2C_Master_Transmit(&hi2c1, dev, reg, 2, 100) != HAL_OK)
        return -1;

    if (HAL_I2C_Master_Receive(&hi2c1, dev, pdata, count, 100) != HAL_OK)
        return -1;

    return 0;
}

int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data) {
    uint8_t buf[3];

    buf[0] = (uint8_t)(index >> 8);
    buf[1] = (uint8_t)(index & 0xFF);
    buf[2] = data;

    if (HAL_I2C_Master_Transmit(&hi2c1, dev, buf, 3, 100) != HAL_OK)
    {
        return -1;
    }

    return 0;
	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */

}

int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data) {
    uint8_t buf[4];

    buf[0] = index >> 8;
    buf[1] = index & 0xFF;
    buf[2] = data >> 8;
    buf[3] = data & 0xFF;

    if (HAL_I2C_Master_Transmit(&hi2c1, dev, buf, 4, 100) != HAL_OK)
        return -1;

    return 0;
}

int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data) {
    uint8_t buf[6];

    buf[0] = index >> 8;
    buf[1] = index & 0xFF;
    buf[2] = (data >> 24) & 0xFF;
    buf[3] = (data >> 16) & 0xFF;
    buf[4] = (data >> 8) & 0xFF;
    buf[5] = data & 0xFF;

    if (HAL_I2C_Master_Transmit(&hi2c1, dev, buf, 6, 100) != HAL_OK)
        return -1;

    return 0;
}

int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *data) {
    uint8_t reg[2];

    reg[0] = (uint8_t)(index >> 8);
    reg[1] = (uint8_t)(index & 0xFF);

    if (HAL_I2C_Master_Transmit(&hi2c1, dev, reg, 2, 100) != HAL_OK)
        return -1;

    if (HAL_I2C_Master_Receive(&hi2c1, dev, data, 1, 100) != HAL_OK)
        return -1;

    return 0;
	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
}

int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *data) {
    uint8_t buf[2];
    uint8_t reg[2];

    reg[0] = (uint8_t)(index >> 8);
    reg[1] = (uint8_t)(index & 0xFF);

    if (HAL_I2C_Master_Transmit(&hi2c1, dev, reg, 2, 100) != HAL_OK)
        return -1;

    if (HAL_I2C_Master_Receive(&hi2c1, dev, buf, 2, 100) != HAL_OK)
        return -1;

    // Itt van a "csere" logika:
    *data = (uint16_t)((buf[0] << 8) | buf[1]);
    return 0;
	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
}

int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *data) {
    uint8_t buf[4];
    uint8_t reg[2];

    reg[0] = index >> 8;
    reg[1] = index & 0xFF;

    if (HAL_I2C_Master_Transmit(&hi2c1, dev, reg, 2, 100) != HAL_OK)
        return -1;

    if (HAL_I2C_Master_Receive(&hi2c1, dev, buf, 4, 100) != HAL_OK)
        return -1;

    *data =
        ((uint32_t)buf[0] << 24) |
        ((uint32_t)buf[1] << 16) |
        ((uint32_t)buf[2] << 8)  |
        ((uint32_t)buf[3]);

    return 0;
}

int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms){

	HAL_Delay(1);
	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
	
	return 0;
}
