/*
 * TOF.c
 *
 *  Created on: 2026. máj. 14.
 *      Author: balin
 */
#include "TOF.h"

volatile uint16_t tof_distance_mm = 0;
float vz_filtered;
float z_prev = 0;
float tof_now = 0;
uint32_t tof_last_time = 0;

extern I2C_HandleTypeDef hi2c1;

void TOF_Parse_Status(uint8_t* tof_data_tmp, uint32_t timestamp_tmp)
{
	uint8_t status = (tof_status & 0x07) ? 1 : 0;

	if(status == 1) {
		TOF_Task_Distance();
		//TOF_Task_Clear();
	}
}

void TOF_Parse_Distance(uint8_t* tof_data_tmp, uint32_t timestamp_tmp)
{

	tof_distance_mm = (tof_data_tmp[0] << 8) | tof_data_tmp[1];
	TOF_Task_Clear();
	ekf_tof_sigma = tof_sigma(tof_distance_mm);
	tof_distance_mm = Get_Filtered_Distance(tof_distance_mm);
	//TOF_Update(tof_distance_mm);
}
float tof_sigma(float z)
{
    if (z < 0.3f) return 0.005f;
    if (z < 1.0f) return 0.015f;
    if (z < 2.0f) return 0.03f;
    return 0.08f;
}

uint16_t Get_Filtered_Distance(uint16_t raw_distance)
{
    // Ha nagyon közel van a földhöz (0-8 cm között), a nyers mérés pontos, nem kell offset
    if (raw_distance < 80)
    {
        return raw_distance;
    }
    // Átmeneti zóna (8 cm és 20 cm között) a hiba fokozatosan növekszik
    else if (raw_distance >= 80 && raw_distance < 200)
    {
        // Lineárisan skálázzuk az offsetet 0-tól 130-ig
        float factor = (float)(raw_distance - 80) / 120.0f;
        return raw_distance + (uint16_t)(factor * 150.0f);
    }
    // Távoli zóna (20 cm felett), ahol a teljes 90mm-es crosstalk offsetre szükség van
    else
    {
        return raw_distance + 150;
    }
}

void TOF_Update(uint16_t raw_mm)
{
    if(raw_mm > 4000)
        return;

    tof_now = (float)(get_us() * INV_CPU_FREQ);
    float dt = tof_now - tof_last_time;
    tof_last_time = tof_now;

    // mm -> m
    float z = raw_mm * 0.001f;

}

void vertical_pos(uint32_t dt)
{

}
