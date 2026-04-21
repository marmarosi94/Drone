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
	#define BETA  1.0f
	#define M_RAD2DEG 57.295779513f
	extern char str[256];

	typedef enum {
	    IMU_DATA_NOT_READY = 0x00,
	    IMU_DATA_READY     = 0x01  // Bit 0 of Register 0x3A
	} IMU_Data_rdy;

	typedef struct {
		IMU_Data_rdy status;       // Register 0x3A (contains Data Ready bit)
	    uint8_t imu_data[14]; // Registers 0x3B to 0x48
	} Imu_raw_data_t;

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

    typedef struct {
        float Roll;
        float Pitch;
        float Yaw;
        float Throttle;
    }Control_t;

    // Accelerometer
	extern Imu_raw_data_t imu_raw;
    extern Vector3 accel;
    extern Vector3 g_ref;
    extern Vector3 gravity_meas;
    extern quaternion quat_flt_orientation;
    // Gyroscope
    extern Vector3 gyro;
    extern Vector3 gyro_Bias;
    extern Vector3 gyro_Sample;
    extern Vector3 gyro_frame;
    extern quaternion quat_gyro;
    extern quaternion quat_delta;
	extern int bias_sample_cnt;
    extern euler_float euler_flt;
    extern Control_t pid_control;
    extern Vector3 velocity;
    extern Vector3 position;

    void IMU_Init(void);
    void IMU_Read_Accel_Gyro(void);
    void IMU_StartRead_Accel_Gyro();
    void IMU_Config_Fast_Mode(void);
    void IMU_Verify_Config();
    void IMU_compute_rotation();
    void IMU_compute_position();
    void IMU_gyro_scale();
    HAL_StatusTypeDef Wait_For_I2C_Complete(uint32_t timeout_ms);
    float vector3_dot(Vector3 a, Vector3 b);
    Vector3 vector3_cross(Vector3 a, Vector3 b);
    Vector3 vector3_normalize(Vector3 v);
    float vector3_length(Vector3 v);
    quaternion quaternion_multiply(quaternion q, quaternion r);
    quaternion quaternion_normalize(quaternion q);
    euler_float quat_to_euler(quaternion );

#endif /* INC_IMU_H_ */
