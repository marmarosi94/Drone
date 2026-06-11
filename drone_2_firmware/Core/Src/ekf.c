/*
 * ekf.c
 *
 *  Created on: 2026. jún. 5.
 *      Author: balin
 */

#include "main.h"

uint32_t ekf_deltatime = 0;
float ekf_deltatime_f = 0.0f;
uint32_t ekf_deltatime_last = 0;

ekf_vertical_state_t vertical_state = {0,0};
ekf_vertical_measurement_t vertical_meas_vec = {0};
ekf_vertical_input_t vertical_input_vec = {0};
float ekf_tof_sigma = 0;
float Vertical_P[2][2] = {
    {1.0f, 0.0f},
    {0.0f, 1.0f}
};
float Vertical_Q[2][2];
//float Vertical_B[2];
float Vertical_R[1] = {0};

ekf_horizontal_state_t horizontal_state = {0,0};
ekf_horizontal_measurement_t horizontal_meas_vec = {0};
ekf_horizontal_input_t horizontal_input_vec = {0};
float ekf_opt_sigma = 0;
float Horizontal_P[4][4] = {
    {1.0f, 0.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 0.0f, 1.0f},
};
float Horizontal_Q[4][4];
//float Horizontal_B[4];
float Horizontal_R[1] = {0};

/*float sa2; // accel noise
float dt2, dt3, dt4, q_pos, q_vel;*/

void print_matrix(const char* name, float *matrix, int rows, int cols) {
    char log_buffer[128]; // Puffer egyetlen sor formázásához
    char row_buffer[512]; // Puffer a teljes mátrix-sor összefűzéséhez

    // Kiírjuk a mátrix nevét az elején
    sprintf(log_buffer, "--- Matrix: %s (%dx%d) ---\r\n", name, rows, cols);
    debug_print(log_buffer);

    for (int i = 0; i < rows; i++) {
        row_buffer[0] = '\0'; // Sor-puffer ürítése minden új sornál

        for (int j = 0; j < cols; j++) {
            // Kiszámoljuk az elem helyét a laposított (1D) memóriaterületen
            float value = matrix[(i * cols) + j];

            // Biztonságos snprintf-et használunk, hogy ne írjuk túl a puffert
            snprintf(log_buffer, sizeof(log_buffer), "%.6f\t", value);

            // Ha még befér a sorba, hozzáfűzzük
            if (strlen(row_buffer) + strlen(log_buffer) < sizeof(row_buffer) - 3) {
                strcat(row_buffer, log_buffer);
            }
        }
        // Sor végén hozzáadjuk a soremelést és kiküldjük a debug portra
        strcat(row_buffer, "\r\n");
        debug_print(row_buffer);
    }
}

void vertical_process_noise(float dt)
{
	float sa2 = 0.4f; // accel noise
	float dt2 = dt*dt;
	float dt3 = dt*dt*dt;
	float dt4 = dt*dt*dt*dt;
	float q_pos = 0.25f * sa2 * dt4;
	float q_vel = sa2 * dt2;

	Vertical_Q[0][0] = 0.25f * sa2 * dt4;
	Vertical_Q[0][1] = 0.5f  * sa2 * dt3;
	Vertical_Q[1][0] = Vertical_Q[0][1];
	Vertical_Q[1][1] = sa2 * dt2;

	Vertical_R[0] = ekf_tof_sigma * ekf_tof_sigma;

	//sprintf(str,"sa2: %f, q_pos: %f, q_vel: %f, dt: %f\r\n", sa2, q_pos, q_vel, dt);
	//debug_print(str);
	//debug_print("\r\n");

}
void vertical_predict(ekf_vertical_state_t *s, ekf_vertical_input_t input, float dt)
{
    float dt2 = 0.5f * dt * dt;
    s->z += s->vz * dt + input.z * dt2;
    s->vz += input.z * dt;
}

void vertical_cov_predict(float P[2][2], float Q[2][2], float dt)
{
    float F[2][2] = {
    	    {1, 0},
    	    {0, 1}
    };

    F[0][1] = dt;
    //print_matrix("F:", *F, 2, 2);
    //print_matrix("P:", *P, 2, 2);
    float FP[2][2] = {0};
    float FPFt[2][2] = {0};

    // FP = F * P
    for(int i=0;i<2;i++){
        for(int j=0;j<2;j++){
            for(int k=0;k<2;k++){
                FP[i][j] += F[i][k] * P[k][j];
            }
        }
    }

    // P = FP * Fᵀ + Q
    for(int i=0;i<2;i++){
        for(int j=0;j<2;j++){
            for(int k=0;k<2;k++){
                FPFt[i][j] += FP[i][k] * F[j][k]; // Fᵀ
            }
            P[i][j] = FPFt[i][j] + Q[i][j];
        }
    }
    //print_matrix("P after:", *P, 2, 2);
    //print_matrix("FPFt after:", *FPFt, 2, 2);
}

void vertical_update(float z_tof, ekf_vertical_state_t *state, float P[2][2], float R)
{
    // Innovation
    float y = z_tof - state->z;

    // Innovation covariance
    float S = P[0][0] + R;

    // Kalman gain
    float K0 = P[0][0] / S;
    float K1 = P[1][0] / S;
	//sprintf(str,"K0: %f, K1 %f\r\n", K0, K1);
 	//debug_print(str);
 	//debug_print("\r\n");
    // State update
    state->z  += K0 * y;
    state->vz += K1 * y;

    // Covariance update
    float P00 = P[0][0];
    float P01 = P[0][1];
    float P10 = P[1][0];
    float P11 = P[1][1];


    P[0][0] = P00 - K0 * P00;
    P[0][1] = P01 - K0 * P01;
    P[1][0] = P10 - K1 * P00;
    P[1][1] = P11 - K1 * P01;

    //print_matrix("P updated:", *P, 2, 2);
}

void horizontal_process_noise(float dt)
{
    float sa2 = 0.4f;

    float dt2 = dt * dt;
    float dt3 = dt2 * dt;
    float dt4 = dt2 * dt2;

    memset(Horizontal_Q, 0, sizeof(Horizontal_Q));

    Horizontal_Q[0][0] = 0.25f * sa2 * dt4;
    Horizontal_Q[0][2] = 0.5f  * sa2 * dt3;

    Horizontal_Q[1][1] = 0.25f * sa2 * dt4;
    Horizontal_Q[1][3] = 0.5f  * sa2 * dt3;

    Horizontal_Q[2][0] = Horizontal_Q[0][2];
    Horizontal_Q[2][2] = sa2 * dt2;

    Horizontal_Q[3][1] = Horizontal_Q[1][3];
    Horizontal_Q[3][3] = sa2 * dt2;

    float z = vertical_state.z;
    float base = ekf_opt_sigma * ekf_opt_sigma;
    // z hatás (enyhébb, nem kvadratikus túlzás)
    float Rz = base * (1.0f + 0.5f * z * z);

    // squal hatás (multiplikatív maradhat)
    float Rs = 1.0f;
    if(optical_data.squal < 10)
        Rs = 4.0f;
    else if(optical_data.squal < 30)
        Rs = 2.0f;

    Horizontal_R[0] = Rz * Rs;
}

void horizontal_predict(ekf_horizontal_state_t *s, ekf_horizontal_input_t input, float dt)
{
    float dt2 = 0.5f * dt * dt;

    s->x += s->vx * dt + input.x * dt2;
    s->y += s->vy * dt + input.y * dt2;

    s->vx += input.x * dt;
    s->vy += input.y * dt;
}

void horizontal_cov_predict(float P[4][4], float Q[4][4], float dt)
{
    float F[4][4] = {0};

    // diagonál + dt kapcsolatok
    for(int i=0;i<4;i++) F[i][i] = 1.0f;

    F[0][2] = dt;
    F[1][3] = dt;

    float FP[4][4] = {0};
    float FPFt[4][4] = {0};

    // FP = F * P
    for(int i=0;i<4;i++){
        for(int j=0;j<4;j++){
            for(int k=0;k<4;k++){
                FP[i][j] += F[i][k] * P[k][j];
            }
        }
    }

    // P = FP * Fᵀ + Q
    for(int i=0;i<4;i++){
        for(int j=0;j<4;j++){
            for(int k=0;k<4;k++){
                FPFt[i][j] += FP[i][k] * F[j][k]; // Fᵀ
            }
            P[i][j] = FPFt[i][j] + Q[i][j];
        }
    }
}

void horizontal_update(float vx_meas, float vy_meas, ekf_horizontal_state_t *state, float P[4][4], float R)
{
    // Innovation
    float y0 = vx_meas - state->vx;
    float y1 = vy_meas - state->vy;

    // S = HPH' + R
    float S00 = P[2][2] + R;
    float S01 = P[2][3];
    float S10 = P[3][2];
    float S11 = P[3][3] + R;

    float det = S00*S11 - S01*S10;

    if(fabsf(det) < 1e-9f)
        return;

    float invS00 =  S11 / det;
    float invS01 = -S01 / det;
    float invS10 = -S10 / det;
    float invS11 =  S00 / det;

    // K = P H' inv(S)
    float K[4][2];

    for(int i=0;i<4;i++)
    {
        K[i][0] = P[i][2] * invS00 + P[i][3] * invS10;
        K[i][1] = P[i][2] * invS01 + P[i][3] * invS11;
    }

    // State update
    state->x  += K[0][0]*y0 + K[0][1]*y1;
    state->y  += K[1][0]*y0 + K[1][1]*y1;
    state->vx += K[2][0]*y0 + K[2][1]*y1;
    state->vy += K[3][0]*y0 + K[3][1]*y1;

    // P = (I-KH)P
    float Pold[4][4];

    memcpy(Pold, P, sizeof(Pold));

    for(int i=0;i<4;i++)
    {
        for(int j=0;j<4;j++)
        {
            P[i][j] =
                Pold[i][j]
                - K[i][0] * Pold[2][j]
                - K[i][1] * Pold[3][j];
        }
    }
}
void ekf_vertical()
{
	uint32_t now = get_us();
	ekf_deltatime = now - ekf_deltatime_last;
	ekf_deltatime_last = now;
	ekf_deltatime_f = (float) ekf_deltatime * INV_CPU_FREQ;

	accel_to_wframe(accel_body, &accel_world);
	vertical_input_vec.z = accel_world.z;

	vertical_meas_vec.z = tof_distance_mm;

	vertical_process_noise(ekf_deltatime_f);
	vertical_predict(&vertical_state, vertical_input_vec, ekf_deltatime_f);
	vertical_cov_predict(Vertical_P, Vertical_Q, ekf_deltatime_f);
	vertical_update(vertical_meas_vec.z, &vertical_state, Vertical_P, Vertical_R[0]);

	//sprintf(str,"Vertical pos: %f, Vertical velocity: %f\r\n", vertical_state.z, vertical_state.vz);
	//debug_print(str);
}

void ekf_horzintal()
{
	horizontal_input_vec.x = accel_world.x;
	horizontal_input_vec.y = accel_world.y;
	horizontal_meas_vec.vx = vx_flow_world;
	horizontal_meas_vec.vy = vy_flow_world;

	horizontal_process_noise(ekf_deltatime_f);
	horizontal_predict(&horizontal_state, horizontal_input_vec, ekf_deltatime_f);
	horizontal_cov_predict(Horizontal_P, Horizontal_Q, ekf_deltatime_f);
	horizontal_update(horizontal_meas_vec.vx, horizontal_meas_vec.vy, &horizontal_state, Horizontal_P, Horizontal_R[0]);

/*
	sprintf(str,"imu x: %.5f,imu y: %.5f\r\n", horizontal_input_vec.x, horizontal_input_vec.y);
	debug_print(str);
	sprintf(str,"opt x: %.5f, opt y: %.5f\r\n", horizontal_meas_vec.vx, horizontal_meas_vec.vy);
	debug_print(str);*/
	//sprintf(str,"Horizontal x: %f, Horizontal y: %f\r\n", horizontal_state.x, horizontal_state.y);
	//debug_print(str);
}
