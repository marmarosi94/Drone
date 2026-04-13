/*
 * PID_CONTROL.h
 *
 *  Created on: Apr 9, 2026
 *      Author: balin
 */

#ifndef INC_PID_CONTROL_H_
#define INC_PID_CONTROL_H_

#include <stdint.h>

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float prevError;
    float antiWindupLimit; // Opcionális: az integrál tag korlátozására
} PID_Axis;

extern PID_Axis pid_control_roll;
extern PID_Axis pid_control_pitch;
extern PID_Axis pid_control_yaw;

extern uint16_t m1; // M1: Jobb-Hátul
extern uint16_t m2; // M2: Jobb-Elöl
extern uint16_t m3; // M3: Bal-Hátul
extern uint16_t m4; // M4: Bal-Elöl

float compute_pid(PID_Axis *pid, float setpoint, float measured, float dt);
void update_motors(float throttle, float roll_pid, float pitch_pid, float yaw_pid);

#endif /* INC_PID_CONTROL_H_ */
