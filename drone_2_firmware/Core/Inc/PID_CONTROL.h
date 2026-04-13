/*
 * PID_CONTROL.h
 *
 *  Created on: Apr 9, 2026
 *      Author: balin
 */

#ifndef INC_PID_CONTROL_H_
#define INC_PID_CONTROL_H_

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

float compute_pid(PID_Axis *pid, float setpoint, float measured, float dt);
void update_motors(float throttle, float roll_pid, float pitch_pid, float yaw_pid);

#endif /* INC_PID_CONTROL_H_ */
