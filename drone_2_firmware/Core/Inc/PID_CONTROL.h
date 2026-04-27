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
    float prevMeasured;      // D-termhez

    float antiWindupLimit;   // integrátor clamp
    float maxOutput;         // PID kimenet limit

    int firstRun;            // 1 (true), ha a következő futás az első
} PID_Axis;

extern PID_Axis pid_roll;
extern PID_Axis pid_pitch;
extern PID_Axis pid_yaw;
extern PID_Axis pid_pos;
extern float gyro_x_smooth;
extern float gyro_y_smooth;
extern float alpha; // Brutális szűrés (0.1 = 90% múlt, 10% új adat)

extern float m1; // M1: Jobb-Hátul
extern float m2; // M2: Jobb-Elöl
extern float m3; // M3: Bal-Hátul
extern float m4; // M4: Bal-Elöl

void PID_Init();
float compute_pid(PID_Axis *pid, float setpoint, float measured, float gyro_rate, float dt);
void update_motors(float throttle, float roll_pid, float pitch_pid, float yaw_pid);

#endif /* INC_PID_CONTROL_H_ */
