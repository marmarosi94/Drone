/*
 * PID_CONTROL.c
 *
 *  Created on: Apr 9, 2026
 *      Author: balin
 */

#include "PID_CONTROL.h"
#include "IMU.h"
#include "MOTOR.h"

PID_Axis pid_control_roll = {78643, 2621, 2621, 0 ,0};
PID_Axis pid_control_pitch = {78643, 2621, 2621, 0 ,0};
PID_Axis pid_control_yaw = {229376, 1310, 0, 0};

float compute_pid(PID_Axis *pid, float setpoint, float measured, float dt) {
    if (dt <= 0.0f) return 0.0f;

    float error = setpoint - measured;

    // P-term
    float p_term = pid->Kp * error;

    // I-term with Anti-Windup
    pid->integral += error * dt;
    // Clamping the integral accumulator
    if (pid->integral > pid->antiWindupLimit) pid->integral = pid->antiWindupLimit;
    else if (pid->integral < -pid->antiWindupLimit) pid->integral = -pid->antiWindupLimit;

    float i_term = pid->Ki * pid->integral;

    // D-term (Consider adding a basic LPF here later)
    float d_term = pid->Kd * ((error - pid->prevError) / dt);
    pid->prevError = error;

    float output = p_term + i_term + d_term;
    return output;
}

void update_motors(float throttle, float roll_pid, float pitch_pid, float yaw_pid)
{
    // X-Config mixer (Motor 1: Jobb-Hátul, 2: Jobb-Elöl, 3: Bal-Hátul, 4: Bal-Elöl)
    // Megjegyzés: A motorok számozása a kódodban PA4(M1), PA6(M2), PA7(M3), PA11(M4)

    uint16_t m1 = throttle - roll_pid + pitch_pid + yaw_pid; // M1: Jobb-Hátul
    uint16_t m2 = throttle - roll_pid - pitch_pid - yaw_pid; // M2: Jobb-Elöl
    uint16_t m3 = throttle + roll_pid + pitch_pid - yaw_pid; // M3: Bal-Hátul
    uint16_t m4 = throttle + roll_pid - pitch_pid + yaw_pid; // M4: Bal-Elöl

    // Beállítás a korábban megírt függvénnyel (vagy közvetlen CCR írással)
    set_motor1_speed(m1);
    set_motor2_speed(m2);
    set_motor3_speed(m3);
    set_motor4_speed(m4);
}
