/*
 * PID_CONTROL.c
 *
 *  Created on: Apr 9, 2026
 *      Author: balin
 */

#include "PID_CONTROL.h"
#include "IMU.h"
#include "MOTOR.h"

PID_Axis pid_roll;
PID_Axis pid_pitch;
PID_Axis pid_yaw;
PID_Axis pid_pos;

float gyro_x_smooth = 0;
float gyro_y_smooth = 0;
float alpha = 0.1f; // Brutális szűrés (0.1 = 90% múlt, 10% új adat)

float m1 = 0.0f; // M1: Jobb-Hátul
float m2 = 0.0f; // M2: Jobb-Elöl
float m3 = 0.0f; // M3: Bal-Hátul
float m4 = 0.0f; // M4: Bal-Elöl

void PID_Init(void)
{
    // -------------------------
    // ROLL (Csillapított teszt értékek)
    // -------------------------
	pid_roll.Kp = 0.6f;     // Visszavéve a stabilitásért
	pid_roll.Ki = 0.1f;     // Nagyon pici I a tartáshoz
	pid_roll.Kd = 0.1f;     // D-tag az oszcilláció ellen

    pid_roll.integral = 0.0f;
    pid_roll.prevMeasured = 0.0f;

    pid_roll.antiWindupLimit = 100.0f;
    pid_roll.maxOutput = 200.0f; // Asztali teszthez elég a 200
    pid_roll.firstRun = 1;

    // -------------------------
    // PITCH (Roll-al szinkronban)
    // -------------------------
    pid_pitch.Kp = 0.6f;
    pid_pitch.Ki = 0.1f;
    pid_pitch.Kd = 0.1f;

    pid_pitch.integral = 0.0f;
    pid_pitch.prevMeasured = 0.0f;

    pid_pitch.antiWindupLimit = 100.0f;
    pid_pitch.maxOutput = 200.0f;
    pid_pitch.firstRun = 1;

    // -------------------------
    // YAW (Legyen nagyon lassú az elején)
    // -------------------------
    pid_yaw.Kp = 0.4f;       // Yaw általában magasabb Kp-t bír
    pid_yaw.Ki = 0.02f;
    pid_yaw.Kd = 0.1f;

    pid_yaw.integral = 0.0f;
    pid_yaw.prevMeasured = 0.0f;

    pid_yaw.antiWindupLimit = 50.0f;
    pid_yaw.maxOutput = 150.0f;
    pid_yaw.firstRun = 1;

    // -------------------------
    // POSITION (KIKAPCSOLVA a teszthez!)
    // -------------------------
    // Amíg a Roll/Pitch oszcillál, a magasság-tartást 0-ra vesszük,
    // különben a motorok maguktól felpörögnek az asztalon!
    pid_pos.Kp = 0.0f;
    pid_pos.Ki = 0.0f;
    pid_pos.Kd = 0.0f;

    pid_pos.integral = 0.0f;
    pid_pos.prevMeasured = 0.0f;

    pid_pos.antiWindupLimit = 0.0f;
    pid_pos.maxOutput = 0.0f;
    pid_pos.firstRun = 1;
}

float compute_pid(PID_Axis *pid, float setpoint, float measured, float gyro_rate, float dt) {

    if (dt <= 0.0f) return 0.0f;

    // --- 1. FIRST RUN LOGIKA ---
    if (pid->firstRun) {
        pid->integral = 0.0f;
        pid->firstRun = 0;
    }

    float error = setpoint - measured;

    // --- 2. P-TAG (Szög alapú hiba) ---
    float p_term = pid->Kp * error;

    // --- 3. D-TAG (Közvetlen szögsebesség alapú csillapítás) ---
    // Mivel a gyro_rate már sebesség (rad/s vagy deg/s), nem kell dt-vel osztani.
    // A negatív előjel azért kell, mert a mozgással ELLENTÉTES irányban akarunk fékezni.
    float d_term = -pid->Kd * gyro_rate;

    // --- 4. I-TAG (Anti-Windup előszámítással) ---
    float integral_candidate = pid->integral + error * dt;

    if (integral_candidate > pid->antiWindupLimit)
        integral_candidate = pid->antiWindupLimit;
    else if (integral_candidate < -pid->antiWindupLimit)
        integral_candidate = -pid->antiWindupLimit;

    float i_term = pid->Ki * integral_candidate;

    // --- 5. KIMENET ÉS LIMITÁLÁS ---
    float output = p_term + i_term + d_term;
    float output_limited = output;

    if (output_limited > pid->maxOutput)
        output_limited = pid->maxOutput;
    else if (output_limited < -pid->maxOutput)
        output_limited = -pid->maxOutput;

    // --- 6. DINAMIKUS ANTI-WINDUP (Clamping) ---
    int saturated = (output != output_limited);

    if (!saturated ||
        (saturated && ((output > pid->maxOutput && error < 0) ||
                       (output < -pid->maxOutput && error > 0)))) {
        pid->integral = integral_candidate;
    }

    return output_limited;
}

void update_motors(float throttle, float roll_pid, float pitch_pid, float yaw_pid)
{
	// A megadott kiosztás alapján:
	    // M1: Bal-Elöl  (+Roll, +Pitch, +Yaw)
	    // M3: Jobb-Elöl (-Roll, +Pitch, -Yaw)
	    // M4: Bal-Hátul (+Roll, -Pitch, -Yaw)
	    // M2: Jobb-Hátul (-Roll, -Pitch, +Yaw)

	    m1 = throttle + roll_pid + pitch_pid + yaw_pid;
	    m3 = throttle - roll_pid + pitch_pid - yaw_pid;
	    m4 = throttle + roll_pid - pitch_pid - yaw_pid;
	    m2 = throttle - roll_pid - pitch_pid + yaw_pid;

	    // Motorok frissítése a fizikai lábakhoz rendelve
	    set_motor1_speed((int16_t)m1); // PA12
	    set_motor2_speed((int16_t)m2); // PA6
	    set_motor3_speed((int16_t)m3); // PA7
	    set_motor4_speed((int16_t)m4); // PB11
}
