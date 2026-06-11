/*
 * PID_CONTROL.c
 *
 *  Created on: Apr 9, 2026
 *      Author: balin
 */

#include "main.h"

PID_t pid_roll;
PID_t pid_pitch;
PID_t pid_yaw;
PID_t pd_height;
PID_t pd_pos;
Vector2 target_angles;
Control_t pid_control = {0};
Vector3 position = {0};
Vector3 velocity = {0};

float m1 = 0.0f;
float m2 = 0.0f;
float m3 = 0.0f;
float m4 = 0.0f;

void PID_Init(void)
{
    // -------------------------
    // ROLL
    // -------------------------
	pid_roll.Kp = 1.2f;
	pid_roll.Ki = 0.2f;
	pid_roll.Kd = 0.3f;

    pid_roll.integral = 0.0f;
    pid_roll.prevMeasured = 0.0f;

    pid_roll.antiWindupLimit = 100.0f;
    pid_roll.maxOutput = 200.0f;
    pid_roll.firstRun = 1;

    // -------------------------
    // PITCH
    // -------------------------
    pid_pitch.Kp = 1.2f;
    pid_pitch.Ki = 0.2f;
    pid_pitch.Kd = 0.3f;

    pid_pitch.integral = 0.0f;
    pid_pitch.prevMeasured = 0.0f;

    pid_pitch.antiWindupLimit = 100.0f;
    pid_pitch.maxOutput = 200.0f;
    pid_pitch.firstRun = 1;

    // -------------------------
    // YAW
    // -------------------------
    pid_yaw.Kp = 1.0f;
    pid_yaw.Ki = 0.2f;
    pid_yaw.Kd = 0.4f; //

    pid_yaw.integral = 0.0f;
    pid_yaw.prevMeasured = 0.0f;

    pid_yaw.antiWindupLimit = 100.0f;
    pid_yaw.maxOutput = 50.0f;
    pid_yaw.firstRun = 1;

    // -------------------------
    // Throttle
    // -------------------------
    pd_height.Kp = 0.5f;
    pd_height.Ki = 0.1f;
    pd_height.Kd = 0.2f; //

    pd_height.integral = 0.0f;
    pd_height.prevMeasured = 0.0f;

    pd_height.antiWindupLimit = 5.0f;
    pd_height.maxOutput = 15.0f;
    pd_height.firstRun = 1;

    // -------------------------
    // Position
    // -------------------------
    pd_pos.Kp = 0.5f;
    pd_pos.Ki = 0.1f;
    pd_pos.Kd = 0.2f; //

    pd_pos.integral = 0.0f;
    pd_pos.prevMeasured = 0.0f;

    pd_pos.antiWindupLimit = 5.0f;
    pd_pos.maxOutput = 15.0f;
    pd_pos.firstRun = 1;

}

float compute_pid(PID_t *pid, float setpoint, float measured, float gyro_rate, float dt) {

    if (dt <= 0.0f) return 0.0f;

    //FIRST RUN logic
    if (pid->firstRun) {
        pid->integral = 0.0f;
        pid->firstRun = 0;
    }

    float error = setpoint - measured;

    //  P (error from angle)
    float p_term = pid->Kp * error;

    // D (direct from gyro only!)
    // Gyroscope give me angular delta
    float d_term = -pid->Kd * gyro_rate;

    // I (Anti-Windup calc)
    float integral_candidate = pid->integral + error * dt;

    if (integral_candidate > pid->antiWindupLimit)
        integral_candidate = pid->antiWindupLimit;
    else if (integral_candidate < -pid->antiWindupLimit)
        integral_candidate = -pid->antiWindupLimit;

    float i_term = pid->Ki * integral_candidate;

    // Output and limit
    float output = p_term + i_term + d_term;
    float output_limited = output;

    if (output_limited > pid->maxOutput)
        output_limited = pid->maxOutput;
    else if (output_limited < -pid->maxOutput)
        output_limited = -pid->maxOutput;

    // dynamic antiwind
    int saturated = (output != output_limited);

    if (!saturated ||
        (saturated && ((output > pid->maxOutput && error < 0) ||
                       (output < -pid->maxOutput && error > 0)))) {
        pid->integral = integral_candidate;
    }

    return output_limited;
}

void update_motors(uint32_t throttle, float roll_pid, float pitch_pid, float yaw_pid)
{
	// Motor mapping:
	    // M1: left-front  (+Roll, +Pitch, +Yaw)
	    // M3: right-front (-Roll, +Pitch, -Yaw)
	    // M4: left-rear (+Roll, -Pitch, -Yaw)
	    // M2: right-rear (-Roll, -Pitch, +Yaw)
		//sprintf(str,"Tht: %.2f\r\n",throttle);
		//debug_print(str);
	    m1 = throttle + roll_pid + pitch_pid + yaw_pid;
	    m3 = throttle - roll_pid + pitch_pid - yaw_pid;
	    m4 = throttle + roll_pid - pitch_pid - yaw_pid;
	    m2 = throttle - roll_pid - pitch_pid + yaw_pid;

	    // Motor update by pins
	    set_motor1_speed((int16_t)m1); // PA12
	    set_motor2_speed((int16_t)m2); // PA6
	    set_motor3_speed((int16_t)m3); // PA7
	    set_motor4_speed((int16_t)m4); // PB11
}
