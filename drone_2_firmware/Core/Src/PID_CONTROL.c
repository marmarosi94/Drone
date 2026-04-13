/*
 * PID_CONTROL.c
 *
 *  Created on: Apr 9, 2026
 *      Author: balin
 */

#include "PID_CONTROL.h"
#include "IMU.h"
#include "MOTOR.h"

PID_Axis pid_control_roll  = {.Kp = 1.8f, .Ki = 0.02f, .Kd = 0.04f, .antiWindupLimit = 150.0f};
PID_Axis pid_control_pitch = {.Kp = 1.8f, .Ki = 0.02f, .Kd = 0.04f, .antiWindupLimit = 150.0f};
PID_Axis pid_control_yaw   = {0.040f, 0.002f, 0.0f , 20.0f};

uint16_t m1 = 0; // M1: Jobb-Hátul
uint16_t m2 = 0; // M2: Jobb-Elöl
uint16_t m3 = 0; // M3: Bal-Hátul
uint16_t m4 = 0; // M4: Bal-Elöl

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
    // Mixer számítása a te egyedi kiosztásod alapján
    m1 = throttle - roll_pid + pitch_pid + yaw_pid; // M1: Bal-Elöl
    m2 = throttle - roll_pid - pitch_pid - yaw_pid; // M2: Jobb-Hátul
    m3 = throttle + roll_pid + pitch_pid - yaw_pid; // M3: Jobb-Elöl
    m4 = throttle + roll_pid - pitch_pid + yaw_pid; // M4: Bal-Hátul

    // Motorok frissítése
    // A belső korlátozás (1000-2000) megvédi az ESC-t a leállástól
    set_motor1_speed((int16_t)m1);
    set_motor2_speed((int16_t)m2);
    set_motor3_speed((int16_t)m3);
    set_motor4_speed((int16_t)m4);
}
