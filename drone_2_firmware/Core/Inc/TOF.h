/*
 * TOF.h
 *
 *  Created on: 2026. máj. 14.
 *      Author: balin
 */

#ifndef INC_TOF_H_
#define INC_TOF_H_

#include "main.h"
#define TOF_I2C_ADDRESS          (0x52)

#define TOF_SOFT_RESET                    0x0000
#define TOF_BOOT_STATE                    0x00E5
#define TOF_IDENTIFICATION_MODEL_ID       0x010F

#define TOF_SYSTEM_START                  0x0087
#define TOF_GPIO_HV_MUX_CTRL              0x0030
#define TOF_SYSTEM_INTERRUPT_CLEAR        0x0086

#define TOF_RESULT_RANGE_STATUS           0x0089
#define TOF_RESULT_DISTANCE               0x0096

extern volatile uint16_t tof_distance_mm;
extern volatile float height;
extern volatile float height_last;
extern volatile float Z_state;
extern float vz_filtered;

void TOF_Parse_Status(uint8_t* tof_data_tmp, uint32_t timestamp_tmp);
void TOF_Parse_Distance(uint8_t* tof_data_tmp, uint32_t timestamp_tmp);
uint16_t Get_Filtered_Distance(uint16_t raw_distance);
void TOF_Update(uint16_t raw_mm);
void vertical_pos(uint32_t dt);
float tof_sigma(float z);

#endif /* INC_TOF_H_ */
