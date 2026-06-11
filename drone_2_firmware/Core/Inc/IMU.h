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

	#define IMU_I2C_ADDRESS 						0xD0 // Example I2C address of the IMU (MPU6050) - 7-bit: 0x68
	// Registers for configuration
	#define PWR_MGMT_1_REG  						0x6B
	#define CONFIG_REG      						0x1A
	#define GYRO_CONFIG_REG 						0x1B
	#define ACCEL_CONFIG_REG 						0x1C
	#define SMPLRT_DIV_REG  						0x19

	#define GYRO_SCALE								0.060975f						//	1/16.4
	#define DEG2RAD 								0.017453f
	#define BIAS_CALIB_SAMPLE_QTY 					1000
	#define ACC_LSB 								4096 							/*AFS_SEL Full Scale Range LSB Sensitivity
																					0 ±2g 16384 LSB/g
																					1 ±4g 8192 LSB/g
																					2 ±8g 4096 LSB/g
																					3 ±16g 2048 LSB/g*/
	#define IMU_DATA_BURST							0x3B
	// MAhony Filter konstansok
	#define KP 1.0f
	#define KI 0.01f//0.005f
	#define RAD2DEG 57.2958f

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
        float w;
        float x;
        float y;
        float z;
    } quaternion;

	typedef struct {
	    int16_t dx;
	    int16_t dy;
	    uint8_t motion;
	    uint8_t quality;
	} OpticalFrame_t;



    // Accelerometer
    extern Vector3 accel;
    extern Vector3 accel_body;
    extern Vector3 accel_world;
    extern Vector3 velocity_imu;
    extern Vector3 position_imu;
    extern Vector3 gravity_meas;
    extern quaternion quat_flt_orientation;
    // Gyroscope
    extern Vector3 gyro;
    extern Vector3 gyro_Bias;
    extern Vector3 gyro_Sample;
    extern Vector3 gyro_frame;
    extern Vector3 gyro_frame_deg;
    extern quaternion quat_gyro;
    extern quaternion quat_delta;
	extern int bias_sample_cnt;
    extern euler_float euler_flt;

    void IMU_Init(void);
    void IMU_Config_Fast_Mode(void);
    void IMU_Verify_Config();
    void IMU_Request_Data(void);
    void IMU_Parse_Data(imu_raw_data_t imu_data_tmp, uint32_t timestamp_tmp);
    void IMU_Calib();
    void IMU_compute_rotation();
    void accel_to_wframe(Vector3 accel_body, Vector3 *accel_world);

    float vector3_dot(Vector3 a, Vector3 b);
    Vector3 vector3_cross(Vector3 a, Vector3 b);
    Vector3 vector3_normalize(Vector3 v);
    float vector3_length(Vector3 v);
    quaternion quaternion_multiply(quaternion q, quaternion r);
    quaternion quaternion_normalize(quaternion q);
    euler_float quat_to_euler(quaternion );

#endif /* INC_IMU_H_ */
