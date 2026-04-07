/*
 * IMU.h
 *
 *  Created on: Apr 1, 2026
 *      Author: balin
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

//#include "main.h"
#include <Q12.h>
#include "COMM.h"
//#include "main.h"


	#define GYRO_SCALE								16.4
	#define DEG2RAD 								0.0174
	#define BIAS_CALIB_SAMPLE_QTY 					500
	#define ACC_LSB 								16384							/*AFS_SEL Full Scale Range LSB Sensitivity
																					0 ±2g 16384 LSB/g
																					1 ±4g 8192 LSB/g
																					2 ±8g 4096 LSB/g
																					3 ±16g 2048 LSB/g*/
	#define GYRO_SCALE_TO_RAD 						0.001065
	#define GYRO_SCALE_TO_RAD_Q12 					4.36
	#define GYRO_RAW_TO_RAD_Q20        				279
	#define SCALE_Q28 								142947

    typedef struct {
        int32_t x;
        int32_t y;
        int32_t z;
    } Vector3;

	extern int bias_sample_cnt;

    // Accelerometer
    extern vec3_q16_t accel;
    extern vec3_q16_t g_ref;
    extern vec3_q16_t g_meas;
    extern quat_q16_t quat_flt_orientation;
    extern quat_q16_t quat_acc;
    // Gyroscope
    extern Vector3 gyro;
    extern Vector3 gyro_Bias;
    extern Vector3 gyro_Sample;
    extern vec3_q16_t gyro_meas;
    extern quat_q16_t quat_gyro;
    extern quat_q16_t quat_delta;

    vec3_q16_t vec_normalize(vec3_q16_t v);
    q16_t vec_dot(vec3_q16_t a, vec3_q16_t b);

    void IMU_Init(void);
    void IMU_Read_Accel_Gyro(void);
    void IMU_Config_Fast_Mode(void);
    void IMU_compute_rotation();
    void IMU_gyro_scale();



#endif /* INC_IMU_H_ */
