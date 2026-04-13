/*
 * IMU.h
 *
 *  Created on: Apr 1, 2026
 *      Author: balin
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

//#include "main.h"
#include "COMM.h"
#include "Math.h"
	#define IMU_I2C_ADDRESS 0xD0 // Example I2C address of the IMU (MPU6050) - 7-bit: 0x68
	// Registers for configuration
	#define PWR_MGMT_1_REG  0x6B
	#define CONFIG_REG      0x1A
	#define GYRO_CONFIG_REG 0x1B
	#define ACCEL_CONFIG_REG 0x1C
	#define SMPLRT_DIV_REG  0x19

	#define GYRO_SCALE								0.060975f						//	1/16.4
	#define DEG2RAD 								0.017453f
	#define BIAS_CALIB_SAMPLE_QTY 					1000
	#define ACC_LSB 								2048							/*AFS_SEL Full Scale Range LSB Sensitivity
																					0 ±2g 16384 LSB/g
																					1 ±4g 8192 LSB/g
																					2 ±8g 4096 LSB/g
																					3 ±16g 2048 LSB/g*/
	// Complementary Filter konstansok
	#define ALPHA 0.98f
	#define BETA  0.02f
	#define MAHONEY_KP  2048   // 0.5 in Q12 (Proportional: how fast it reacts)
	#define MAHONEY_KI  40     // 0.01 in Q12 (Integral: how fast it "learns" bias)

	extern char str[256];

    typedef struct {
        float x;
        float y;
        float z;
    } Vector3;

    typedef struct {
        float roll;
        float pitch;
        float yaw;
    } euler_float;

    typedef struct {
        float x;
        float y;
        float z;
        float w;
    } quaternion;


	extern int bias_sample_cnt;

    // Accelerometer
    extern Vector3 accel;
    extern Vector3 g_ref;
    extern Vector3 g_meas;
    extern quaternion quat_flt_orientation;
    extern quaternion quat_acc;
    // Gyroscope
    extern Vector3 gyro;
    extern Vector3 gyro_flt;
    extern Vector3 gyro_Bias;
    extern Vector3 gyro_Sample;
    extern Vector3 gyro_meas;
    extern quaternion quat_gyro;
    extern quaternion quat_delta;
    extern euler_float euler_flt;

    euler_float quat_to_euler(quaternion );
    quaternion quaternion_multiply(quaternion q, quaternion r);
    quaternion quaternion_normalize(quaternion q);

    float vector3_dot(Vector3 a, Vector3 b);
    Vector3 vector3_cross(Vector3 a, Vector3 b);
    Vector3 vector3_normalize(Vector3 v);

    void IMU_Init(void);
    void IMU_Read_Accel_Gyro(void);
    void IMU_Config_Fast_Mode(void);
    void IMU_Verify_Config();
    void IMU_compute_rotation();
    void IMU_gyro_scale();
    HAL_StatusTypeDef Wait_For_I2C_Complete(uint32_t timeout_ms);


#endif /* INC_IMU_H_ */
