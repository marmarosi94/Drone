/*
 * ekf.h
 *
 *  Created on: 2026. jún. 5.
 *      Author: balin
 */

#ifndef INC_EKF_H_
#define INC_EKF_H_

//////////////////////Horizontal EKF matrices, vectors//////////////////////

typedef struct {
    float z;      // position z (world)
    float vz;      // position z (world)
} ekf_vertical_state_t;

typedef struct {
    float z;      // position z (world)
} ekf_vertical_measurement_t;

typedef struct {
    float z;      // acceleration z from IMU (world)
} ekf_vertical_input_t;

//////////////////////Vertical EKF matrices, vectors//////////////////////

typedef struct {
    float x;      // position X (world)
    float y;      // position Y (world)
    float vx;     // velocity X (world)
    float vy;     // velocity Y (world)
} ekf_horizontal_state_t;

typedef struct {
    float vx;      // position X (world)
    float vy;      // position Y (world)
} ekf_horizontal_measurement_t;

typedef struct {
    float x;      // acceleration X (world)
    float y;      // acceleration Y (world)
} ekf_horizontal_input_t;

extern ekf_vertical_state_t vertical_state;
extern ekf_vertical_input_t vertical_input_vec;
extern ekf_horizontal_state_t horizontal_state;
extern float ekf_tof_sigma;

void ekf_vertical();
void ekf_horzintal();

#endif /* INC_EKF_H_ */
