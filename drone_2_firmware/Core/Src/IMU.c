/*
 * IMU.c
 *
 *  Created on: Apr 1, 2026
 *      Author: balin
 */
#include "main.h"
#include <stdbool.h>


// ==== GLOBAL VARIABLE DEFINITIONS ====
int bias_sample_cnt = 0;
uint8_t gyro_is_calibrated = 0;
imu_raw_data_t imu_raw={0};
uint32_t  t_calib = 0;
static uint32_t imu_last_timestamp = 0;

// Accelerometer
Vector3 accel = {0};
Vector3 accel_body = {0};
Vector3 accel_world = {0};
Vector3 velocity_imu = {0};
Vector3 position_imu = {0};
Vector3 gyro_Bias = {0};
Vector3 gyro_Sample = {0};
Vector3 gyro_frame = {0};
Vector3 gyro_frame_deg = {0};
Vector3 gyro;
Vector3 gravity_meas = {0,0,0};
Vector3 gyro_bias_integral = {0, 0, 0};
quaternion quat_gyro = {1,0,0,0};
quaternion quat_delta = {1,0,0,0};
quaternion quat_flt_orientation = {0};
euler_float euler_flt = {0};

// Function to initialize the IMU
void IMU_Init(void) {
    uint8_t data;
    HAL_StatusTypeDef status;

    //Wakeup (PWR_MGMT_1 register 0x6B -> 0x00)
    data = 0x00;
    status = HAL_I2C_Mem_Write(&hi2c1, IMU_I2C_ADDRESS, 0x6B, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
    if (status != HAL_OK) {
        debug_print("Error: IMU Wake-up failed\r\n");
        return;
    }
    delay_ms(100); // wait for osc

    //WHO_AM_I check (Regiszter 0x75)
    for (uint8_t i = 0; i < 10; i++) {
        status = HAL_I2C_Mem_Read(&hi2c1, IMU_I2C_ADDRESS, 0x75, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);

        if (status == HAL_OK && data == 0x68) {
            debug_print("IMU FOUND! (0x68)\r\n");
            IMU_Config_Fast_Mode(); // config
            return;
        }

        debug_print("IMU NOT found, retrying...\r\n");
        delay_ms(50);
    }
    debug_print("Error: IMU Init failed!\r\n");
}

void IMU_Config_Fast_Mode(void) {
    uint8_t data;
    uint8_t i2c_addr = IMU_I2C_ADDRESS;

    // Sample Rate Divider (Register 0x19)
    // 0x00 = 1kHz inner samplerate
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, i2c_addr, 0x19, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);

    // DLPF (Low Pass Filter) Register 0x1A)
    // 0x03 = ~42Hz cutoff
    data = 0x03;
    HAL_I2C_Mem_Write(&hi2c1, i2c_addr, 0x1A, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);

    // Gyro Full Scale Range (Regiszter 0x1B)
    // 0x18 = ±2000 deg/s (kell a Wizard sebességéhez)
    data = 0x18;
    HAL_I2C_Mem_Write(&hi2c1, i2c_addr, 0x1B, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);

    // Accel Full Scale Range (Regiszter 0x1C)
    // 0x10 = ±8g
    data = 0x10;
    HAL_I2C_Mem_Write(&hi2c1, i2c_addr, 0x1C, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
    //INT_ENABLE
    data = 0x01;
    HAL_I2C_Mem_Write(&hi2c1, i2c_addr, 0x38, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);

    debug_print("IMU Fast Mode configured.\r\n");
    IMU_Verify_Config();
}

void IMU_Verify_Config(void) {
    uint8_t read_data;
    uint8_t i2c_addr = IMU_I2C_ADDRESS;
    HAL_StatusTypeDef status;
    bool error_found = false;

    // Exptected register values
    struct {
        uint8_t reg;
        uint8_t expected;
        char* name;
    } config_steps[] = {
        {0x1A, 0x03, "DLPF"},
        {0x1B, 0x18, "Gyro Range"},
        {0x1C, 0x10, "Accel Range"},
        {0x19, 0x00, "Sample Rate"}
    };

    debug_print("--- IMU Verification Start ---\r\n");

    for (int i = 0; i < 4; i++) {
        status = HAL_I2C_Mem_Read(&hi2c1, i2c_addr, config_steps[i].reg, I2C_MEMADD_SIZE_8BIT, &read_data, 1, 100);

        if (status != HAL_OK) {
        	sprintf(str,"Error: Could not read %s (0x%02X)\r\n", config_steps[i].name, config_steps[i].reg);
            debug_print(str);
            error_found = true;
            continue;
        }

        if (read_data == config_steps[i].expected) {
        	sprintf(str,"OK: %s set to 0x%02X\r\n", config_steps[i].name, read_data);
        	 debug_print(str);
        } else {
        	sprintf(str,"FAIL: %s is 0x%02X (Expected: 0x%02X)\r\n", config_steps[i].name, read_data, config_steps[i].expected);
            error_found = true;
            debug_print(str);
        }
    }

    if (!error_found) {
        debug_print("IMU Configuration Verified Successfully!\r\n");
    } else {
        debug_print("IMU Configuration FAILED!\r\n");
    }

    debug_print("--- IMU Verification End ---\r\n");
}

void IMU_Calib(){
    while(1)
    {
        uint32_t now = get_us();

        if ((uint32_t)(now - t_calib) >= LOOP1_CYCLES)
        {
        	IMU_Task();
        	I2C_Task();
            t_calib += LOOP1_CYCLES;
        }
        if(SENSOR_Process())
        {
			gyro_Sample.x += gyro.x;
			gyro_Sample.y += gyro.y;
			gyro_Sample.z += gyro.z;
			bias_sample_cnt++;
        }
        if (bias_sample_cnt >= BIAS_CALIB_SAMPLE_QTY)
        {
            gyro_Bias.x =  (float)gyro_Sample.x / bias_sample_cnt;
            gyro_Bias.y = (float)gyro_Sample.y / bias_sample_cnt;
            gyro_Bias.z = (float)gyro_Sample.z / bias_sample_cnt;
            debug_print("Gyro calibrated!\r\n");
            return;
        }
    }
}

// Function to read accelerometer and gyroscope data
void IMU_Parse_Data(imu_raw_data_t imu_data_tmp, uint32_t timestamp_tmp){

	accel.x = (int16_t)((imu_data_tmp.imu_data[0] << 8) | imu_data_tmp.imu_data[1]);
	accel.y = (int16_t)((imu_data_tmp.imu_data[2] << 8) | imu_data_tmp.imu_data[3]);
	accel.z = (int16_t)((imu_data_tmp.imu_data[4] << 8) | imu_data_tmp.imu_data[5]);

	gyro.x  = (int16_t)((imu_data_tmp.imu_data[8] << 8) | imu_data_tmp.imu_data[9]);
	gyro.y  = (int16_t)((imu_data_tmp.imu_data[10] << 8) | imu_data_tmp.imu_data[11]);
	gyro.z  = (int16_t)((imu_data_tmp.imu_data[12] << 8) | imu_data_tmp.imu_data[13]);

    uint32_t dt_cycles = timestamp_tmp - imu_last_timestamp;
    imu_last_timestamp = timestamp_tmp;
    imu_deltatime = (float)dt_cycles * INV_CPU_FREQ;
}

void IMU_compute_rotation() {

	//Accel alignement and raw datas
	accel_body.x = (float)accel.y /  ACC_LSB;
	accel_body.y = (float)accel.x / -ACC_LSB;
	accel_body.z = (float)accel.z /  ACC_LSB;
/*
    float ax_vaz =  ay;
    float ay_vaz = -ax;
    float az_vaz =  az;
*/
    gravity_meas.x = accel_body.x;
    gravity_meas.y = accel_body.y;
    gravity_meas.z = accel_body.z;
	gravity_meas = vector3_normalize(gravity_meas);

	//Gyro alignement and raw datas-
	float gx = ((float)gyro.x - gyro_Bias.x) * GYRO_SCALE;
	float gy = ((float)gyro.y - gyro_Bias.y) * GYRO_SCALE;
	float gz = ((float)gyro.z - gyro_Bias.z) * GYRO_SCALE;

	//Match the accelero and gyro axis
	float gx_vaz =  gy;
	float gy_vaz = -gx;
	float gz_vaz =  gz;

	gyro_frame_deg.x = gx_vaz;
	gyro_frame_deg.y = gy_vaz;
	gyro_frame_deg.z = gz_vaz;

	gyro_frame.x = gx_vaz * DEG2RAD;
	gyro_frame.y = gy_vaz * DEG2RAD;
	gyro_frame.z = gz_vaz * DEG2RAD;

	//Mahony logic
	Vector3 gyro_g_ref = {0};

	gyro_g_ref.x = 2.0f * (quat_gyro.x * quat_gyro.z - quat_gyro.w * quat_gyro.y);
	gyro_g_ref.y = 2.0f * (quat_gyro.w * quat_gyro.x + quat_gyro.y * quat_gyro.z);
	gyro_g_ref.z = quat_gyro.w * quat_gyro.w
				 - quat_gyro.x * quat_gyro.x
				 - quat_gyro.y * quat_gyro.y
				 + quat_gyro.z * quat_gyro.z;

	Vector3 error;

	error.x = gravity_meas.y * gyro_g_ref.z - gravity_meas.z * gyro_g_ref.y;
	error.y = gravity_meas.z * gyro_g_ref.x - gravity_meas.x * gyro_g_ref.z;
	error.z = gravity_meas.x * gyro_g_ref.y - gravity_meas.y * gyro_g_ref.x;

	gyro_bias_integral.x += error.x * KI * imu_deltatime;
	gyro_bias_integral.y += error.y * KI * imu_deltatime;
	gyro_bias_integral.z += error.z * KI * imu_deltatime;

	gyro_frame.x += KP * error.x + gyro_bias_integral.x;
	gyro_frame.y += KP * error.y + gyro_bias_integral.y;
	gyro_frame.z += KP * error.z + gyro_bias_integral.z;


	//Quaternion integration (shortest way)
	float angle = vector3_length(gyro_frame) * imu_deltatime;

	if (angle > 1e-6f) {
		Vector3 axis = vector3_normalize(gyro_frame);
		float s = sinf(angle * 0.5f);
		quat_delta.w = cosf(angle * 0.5f);
		quat_delta.x = axis.x * s;
		quat_delta.y = axis.y * s;
		quat_delta.z = axis.z * s;
	} else {
		quat_delta.w = 1;
		quat_delta.x = 0;
		quat_delta.y = 0;
		quat_delta.z = 0;
	}
	quat_gyro = quaternion_multiply(quat_gyro, quat_delta);
	quat_flt_orientation = quaternion_normalize(quat_gyro);
	quat_gyro = quat_flt_orientation;
}

void accel_to_wframe(Vector3 accel_body, Vector3 *accel_world)
{
    float qw = quat_flt_orientation.w;
    float qx = quat_flt_orientation.x;
    float qy = quat_flt_orientation.y;
    float qz = quat_flt_orientation.z;

    // normalize quaternion
    float norm = sqrtf(qw*qw + qx*qx + qy*qy + qz*qz);
    qw/=norm; qx/=norm; qy/=norm; qz/=norm;

    float ax = accel_body.x;
    float ay = accel_body.y;
    float az = accel_body.z;

    float ix =  qw * ax + qy * az - qz * ay;
    float iy =  qw * ay + qz * ax - qx * az;
    float iz =  qw * az + qx * ay - qy * ax;
    float iw = -qx * ax - qy * ay - qz * az;

    accel_world->x = ix * qw + iw * -qx + iy * -qz - iz * -qy;
    accel_world->y = iy * qw + iw * -qy + iz * -qx - ix * -qz;
    accel_world->z = iz * qw + iw * -qz + ix * -qy - iy * -qx;

    // gravity removal (clean version)
    accel_world->z -= 1.0f;
}

// Dot Product for vectors
float vector3_dot(Vector3 a, Vector3 b) {
    return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
}

//Cross Product for vectors
Vector3 vector3_cross(Vector3 a, Vector3 b) {
    Vector3 result;
    result.x = (a.y * b.z) - (a.z * b.y);
    result.y = (a.z * b.x) - (a.x * b.z);
    result.z = (a.x * b.y) - (a.y * b.x);
    return result;
}

// Normalize
Vector3 vector3_normalize(Vector3 v) {
    float magSq = (v.x * v.x) + (v.y * v.y) + (v.z * v.z);

    if (magSq > 0.000001f) { // threshold
        float invMag = 1.0f / sqrtf(magSq);
        return (Vector3){v.x * invMag, v.y * invMag, v.z * invMag};
    }

    return (Vector3){0.0f, 0.0f, 0.0f}; // If the lenght is 0
}
float vector3_length(Vector3 v) {
    return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}
quaternion quaternion_multiply(quaternion q, quaternion r) {
	quaternion res;

	// W
	res.w = q.w * r.w - q.x * r.x - q.y * r.y - q.z * r.z;
	// X
	res.x = q.w * r.x + q.x * r.w + q.y * r.z - q.z * r.y;
	// Y
	res.y = q.w * r.y - q.x * r.z + q.y * r.w + q.z * r.x;
	// Z
	res.z = q.w * r.z + q.x * r.y - q.y * r.x + q.z * r.w;
	return res;
}

quaternion quaternion_normalize(quaternion q) {
    float magSq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;

    if (magSq > 0.000001f) {
        float invMag = 1.0f / sqrtf(magSq);
        return (quaternion){
            .w = q.w * invMag,
            .x = q.x * invMag,
            .y = q.y * invMag,
            .z = q.z * invMag
        };
    }
    return (quaternion){ .w = 1.0f, .x = 0.0f, .y = 0.0f, .z = 0.0f };
}

euler_float quat_to_euler(quaternion q)
{
    euler_float euler;

    // qha to normalized
    float t0 = 2.0f * (q.w * q.x + q.y * q.z);
    float t1 = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    euler.pitch = atan2f(t0, t1) * RAD2DEG;

    float t2 = 2.0f * (q.w * q.y - q.z * q.x);
    t2 = fmaxf(-1.0f, fminf(1.0f, t2));
    euler.roll = asinf(t2) * RAD2DEG;

    float t3 = 2.0f * (q.w * q.z + q.x * q.y);
    float t4 = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    euler.yaw = atan2f(t3, t4) * RAD2DEG;

    return euler;
}
