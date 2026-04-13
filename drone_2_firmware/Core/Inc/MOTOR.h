/*
 * MOTOR.h
 *
 *  Created on: Apr 9, 2026
 *      Author: balin
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "main.h"

#define MIN_THROTTLE 1010
#define MAX_THROTTLE 1350


void esc_init();
void set_all_motors(int16_t throttle);
void set_motor1_speed(int16_t throttle);
void set_motor2_speed(int16_t throttle);
void set_motor3_speed(int16_t throttle);
void set_motor4_speed(int16_t throttle);

#endif /* INC_MOTOR_H_ */
