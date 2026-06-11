/*
 * Optic.h
 *
 *  Created on: 2026. máj. 7.
 *      Author: balin
 */

#ifndef INC_OPTIC_H_
#define INC_OPTIC_H_

#include "main.h"

#define PMW3901_REG_ID                0x00
#define PMW3901_REG_DATA_READY        0x02
#define PMW3901_REG_MOTION_BURST      0x16
#define PMW3901_REG_POWER_UP_RESET    0x3A
#define PMW3901_REG_SHUTDOWN		  0x3B
#define REFERENCE_HEIGHT 			  600
#define FLOW_SCALER 				  600.0f
#define DELTA_X_HIGH				  0x04
#define DELTA_X_LOW 				  0x03
#define DELTA_Y_HIGH				  0x06
#define DELTA_Y_LOW 				  0x05

typedef struct {
	float x;
	float y;
} Vector2;

typedef struct {
    uint8_t motion;
    int16_t deltaX;
    int16_t deltaY;
    uint8_t squal;
} PMW3901_Data_t;

extern PMW3901_Data_t optical_data;
extern Vector2 optical_pos;
extern uint8_t opt_read_state;

extern float vx_flow_world;
extern float vy_flow_world;

void PMW3901_Write(uint8_t reg, uint8_t value);
uint8_t PMW3901_Read(uint8_t reg);
uint8_t PMW3901_Read_reg(uint8_t reg);
uint8_t PMW3901_Init(void);
void PMW3901_SecretSauce(void);
void optical_request_motion(void);
void optical_parse(uint8_t* raw);
void body_to_world(int dx, int dy,  float yaw,  float dt,  float height,   float scale,   float *vx_world,  float *vy_world);
void FLOW_Update(int16_t dx, int16_t dy, float dt);

#endif /* INC_OPTIC_H_ */
