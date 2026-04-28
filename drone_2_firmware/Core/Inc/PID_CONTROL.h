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
    float prevMeasured;      // D-term

    float antiWindupLimit;   // integrator clamp
    float maxOutput;         // PID outpuit limit

    int firstRun;            // 1 (true)
} PID_Axis;

extern PID_Axis pid_roll;
extern PID_Axis pid_pitch;
extern PID_Axis pid_yaw;
extern PID_Axis pid_pos;

extern float m1; 			// M1: right-read
extern float m2; 			// M2: right-front
extern float m3; 			// M3: left-rear
extern float m4; 			// M4: left-front

void PID_Init();
float compute_pid(PID_Axis *pid, float setpoint, float measured, float gyro_rate, float dt);
void update_motors(float throttle, float roll_pid, float pitch_pid, float yaw_pid);

#endif /* INC_PID_CONTROL_H_ */
