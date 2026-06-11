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
	float Roll;
	float Pitch;
	float Yaw;
	int32_t Throttle;
	int32_t Height;
}Control_t;

typedef struct {
    float Kp;
    float Ki;
    float Kd;

    float integral;
    float prevMeasured;      // D-term

    float antiWindupLimit;   // integrator clamp
    float maxOutput;         // PID outpuit limit

    int firstRun;            // 1 (true)
} PID_t;

extern PID_t pid_roll;
extern PID_t pid_pitch;
extern PID_t pid_yaw;
extern PID_t pd_height;
extern PID_t pd_pos;
extern Control_t pid_control;
extern Control_t pid_target;
extern Vector3 position;
extern Vector3 velocity;

extern float m1; 			// M1: right-read
extern float m2; 			// M2: right-front
extern float m3; 			// M3: left-rear
extern float m4; 			// M4: left-front

void PID_Init();
float compute_pid(PID_t *pid, float setpoint, float measured, float gyro_rate, float dt);
void update_motors(uint32_t throttle, float roll_pid, float pitch_pid, float yaw_pid);

#endif /* INC_PID_CONTROL_H_ */
