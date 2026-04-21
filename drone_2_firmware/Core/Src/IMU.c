/*
 * IMU.c
 *
 *  Created on: Apr 1, 2026
 *      Author: balin
 */
#include "IMU.h"
#include <stdbool.h>


// ==== GLOBAL VARIABLE DEFINITIONS ====
int bias_sample_cnt = 0;
I2C_State_t imu_i2c_state;
uint8_t gyro_is_calibrated = 0;
Imu_raw_data_t imu_raw={0};

// Accelerometer
Vector3 accel = {0};
Vector3 gyro_Bias = {0};
Vector3 gyro_Sample = {0};
Vector3 gyro_frame = {0};
Vector3 gyro;
Vector3 g_ref = {0,0,1};
Vector3 gravity_meas = {0,0,0};
Vector3 gyro_bias_integral = {0, 0, 0}; // Az integrált hiba tárolója
float KI = 0.001f;                      // Integrál erősítés (nagyon kicsi érték kell ide)
quaternion quat_gyro = {0,0,0,1};
quaternion quat_delta = {0,0,0,1};
quaternion quat_flt_orientation = {0};
euler_float euler_flt = {0};
Control_t pid_control = {0};
Vector3 position = {0};
Vector3 velocity = {0};

// Function to initialize the IMU
void IMU_Init(void) {
    uint8_t data;
    HAL_StatusTypeDef status;

    // 1. Ébresztés (PWR_MGMT_1 regiszter 0x6B -> 0x00)
    data = 0x00;
    status = HAL_I2C_Mem_Write(&hi2c1, IMU_I2C_ADDRESS, 0x6B, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
    if (status != HAL_OK) {
        debug_print("Error: IMU Wake-up failed\r\n");
        return;
    }
    delay_ms(100); // Várjunk az oszcillátor stabilizálódására

    // 2. WHO_AM_I ellenőrzése (Regiszter 0x75)
    for (uint8_t i = 0; i < 10; i++) {
        status = HAL_I2C_Mem_Read(&hi2c1, IMU_I2C_ADDRESS, 0x75, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);

        if (status == HAL_OK && data == 0x68) {
            debug_print("IMU FOUND! (0x68)\r\n");
            IMU_Config_Fast_Mode(); // Konfiguráció futtatása
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
    //uint8_t i2c_addr = (IMU_I2C_ADDRESS << 1);

    // DLPF (Low Pass Filter) beállítása (Regiszter 0x1A)
    // 0x03 = ~42Hz cutoff (kiszűri a motorvibrációt)
    data = 0x03;
    HAL_I2C_Mem_Write(&hi2c1, i2c_addr, 0x1A, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);

    // Gyro Full Scale Range (Regiszter 0x1B)
    // 0x18 = ±2000 deg/s (kell a Wizard sebességéhez)
    data = 0x18;
    HAL_I2C_Mem_Write(&hi2c1, i2c_addr, 0x1B, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);

    // Accel Full Scale Range (Regiszter 0x1C)
    // 0x18 = ±16g
    data = 0x18;
    HAL_I2C_Mem_Write(&hi2c1, i2c_addr, 0x1C, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);

    // Sample Rate Divider (Regiszter 0x19)
    // 0x00 = 1kHz belső mintavételezés
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, i2c_addr, 0x19, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);

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

    // Regiszterek és elvárt értékeik listája
    struct {
        uint8_t reg;
        uint8_t expected;
        char* name;
    } config_steps[] = {
        {0x1A, 0x03, "DLPF"},
        {0x1B, 0x18, "Gyro Range"},
        {0x1C, 0x18, "Accel Range"},
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

// Function to read accelerometer and gyroscope data
void IMU_Read_Accel_Gyro(void) {
    if(dma_read_complete == I2C_IDLE) {
        I2C_Read_DMA(IMU_I2C_ADDRESS, 0x3A, (uint8_t *)&imu_raw, 15);
    }
    if(dma_read_complete == I2C_BUSY) {
    	return;
    }
    if(dma_read_complete == I2C_COMPLETE) {
    	if(imu_raw.status == IMU_DATA_READY)
		{
			// Note: imu_data[6-7] is Temperature, which is why we skip to [8]
			accel.x = (int16_t)((imu_raw.imu_data[0] << 8) | imu_raw.imu_data[1]);
			accel.y = (int16_t)((imu_raw.imu_data[2] << 8) | imu_raw.imu_data[3]);
			accel.z = (int16_t)((imu_raw.imu_data[4] << 8) | imu_raw.imu_data[5]);

			gyro.x  = (int16_t)((imu_raw.imu_data[8] << 8) | imu_raw.imu_data[9]);
			gyro.y  = (int16_t)((imu_raw.imu_data[10] << 8) | imu_raw.imu_data[11]);
			gyro.z  = (int16_t)((imu_raw.imu_data[12] << 8) | imu_raw.imu_data[13]);
		}
        dma_read_complete = I2C_IDLE;
    }
}

void IMU_compute_rotation() {
    // Accelerometer data
	gravity_meas.x = (float)accel.x / ACC_LSB;
	gravity_meas.y = (float)accel.y / ACC_LSB;
	gravity_meas.z = (float)accel.z / ACC_LSB;

    //Aceleroy normalization
    gravity_meas = vector3_normalize(gravity_meas);

    // Gyro calibration
    if(gyro_is_calibrated == 0) {
        gyro_Sample.x += gyro.x;
        gyro_Sample.y += gyro.y;
        gyro_Sample.z += gyro.z;
        bias_sample_cnt++;
        if (BIAS_CALIB_SAMPLE_QTY <= bias_sample_cnt) gyro_is_calibrated = 1;
    }

    if(gyro_is_calibrated == 1) {
        gyro_Bias.x = (float) gyro_Sample.x / bias_sample_cnt;
        gyro_Bias.y = (float) gyro_Sample.y / bias_sample_cnt;
        gyro_Bias.z = (float) gyro_Sample.z / bias_sample_cnt;
        gyro_is_calibrated = 2;
    }

    if(gyro_is_calibrated == 2) {
            // Gyro rotation
    		gyro_frame.x = -((float)gyro.x - gyro_Bias.x) * GYRO_SCALE;
    		gyro_frame.y =  ((float)gyro.y - gyro_Bias.y) * GYRO_SCALE;
    		gyro_frame.z =  ((float)gyro.z - gyro_Bias.z) * GYRO_SCALE;

            // MAHONY LOGIC: Gravity quaternion prediction from gyro // quaternion -> rotation matrix Z
            Vector3 gyro_g_ref= {0};
            gyro_g_ref.x = 2.0f * (quat_gyro.x * quat_gyro.z - quat_gyro.w * quat_gyro.y);
            gyro_g_ref.y = 2.0f * (quat_gyro.w * quat_gyro.x + quat_gyro.y * quat_gyro.z);
            gyro_g_ref.z = quat_gyro.w * quat_gyro.w - quat_gyro.x * quat_gyro.x - quat_gyro.y * quat_gyro.y + quat_gyro.z * quat_gyro.z;

            // Error of frame and world gravity orientation
            Vector3 error = {0};
            error.x = (gravity_meas.y * gyro_g_ref.z - gravity_meas.z * gyro_g_ref.y);
            error.y = (gravity_meas.z * gyro_g_ref.x - gravity_meas.x * gyro_g_ref.z);
            error.z = (gravity_meas.x * gyro_g_ref.y - gravity_meas.y * gyro_g_ref.x);

            //deltatime = imu_deltatime_us() * 0.000001f;
            deltatime = 0.001f;

            // error * KI * deltatime
            gyro_bias_integral.x += error.x * KI * deltatime;
            gyro_bias_integral.y += error.y * KI * deltatime;
            gyro_bias_integral.z += error.z * KI * deltatime;

            //Gravity correction
            gyro_frame.x += BETA * error.x + gyro_bias_integral.x;
            gyro_frame.y += BETA * error.y + gyro_bias_integral.y;
            gyro_frame.z += BETA * error.z + gyro_bias_integral.z;

            float half_rad = 0.5f * DEG2RAD * deltatime;

            // Quat delta
			quat_delta.w = 1.0f;
			quat_delta.x = gyro_frame.x * half_rad;
			quat_delta.y = gyro_frame.y * half_rad;
			quat_delta.z = gyro_frame.z * half_rad;

            // Integrálás (Fontos a sorrend: régi * delta)
            quat_gyro = quaternion_multiply(quat_gyro, quat_delta);
            quat_flt_orientation = quaternion_normalize(quat_gyro);
            //Feedback
            quat_gyro = quat_flt_orientation;
      }
}
void IMU_compute_position(){
	    // Gravity compensation (with normalized values)
	    Vector3 lin_acc_body;
	    lin_acc_body.x = gravity_meas.x - g_ref.x;
	    lin_acc_body.y = gravity_meas.y - g_ref.y;
	    lin_acc_body.z = gravity_meas.z - g_ref.z;

	    //Rotation to world frame
	    // A quat_flt_orientation kvaternióval forgatjuk a lin_acc_body vektort
	    Vector3 lin_acc_earth;

	    // Hatékony vektor-kvaternió forgatás (v' = q * v * q^-1)
	    float qw = quat_flt_orientation.w;
	    float qx = quat_flt_orientation.x;
	    float qy = quat_flt_orientation.y;
	    float qz = quat_flt_orientation.z;
	    Vector3 q_vec = {qx, qy, qz};
	    Vector3 t = vector3_cross(q_vec, lin_acc_body);
	    t.x *= 2.0f;
	    t.y *= 2.0f;
	    t.z *= 2.0f;
	    lin_acc_earth.x = lin_acc_body.x + qw * t.x + (qy * t.z - qz * t.y);
	    lin_acc_earth.y = lin_acc_body.y + qw * t.y + (qz * t.x - qx * t.z);
	    lin_acc_earth.z = lin_acc_body.z + qw * t.z + (qx * t.y - qy * t.x);

	    // Integrationto velocity (m/s)
	    // 9.81f szorzó kell, ha a gyorsulásmérőd 1g-ben mér
	    float acc_m_s2 = 9.81f;
	    float deadband = 0.05f; // Zajszűrés: csak ennél nagyobb gyorsulást integrálunk

	    if (fabs(lin_acc_earth.x) > deadband) velocity.x += lin_acc_earth.x * acc_m_s2 * deltatime;
	    else velocity.x *= 0.98f; // Damping: lassú megállás ha nincs mozgás

	    if (fabs(lin_acc_earth.y) > deadband) velocity.y += lin_acc_earth.y * acc_m_s2 * deltatime;
	    else velocity.y *= 0.98f;

	    if (fabs(lin_acc_earth.z) > deadband) velocity.z += lin_acc_earth.z * acc_m_s2 * deltatime;
	    else velocity.z *= 0.98f;

	    // Integration to position
	    position.x += velocity.x * deltatime;
	    position.y += velocity.y * deltatime;
	    position.z += velocity.z * deltatime;

}
// Segédfüggvény a várakozáshoz, hogy ne ismételjük a kódot
HAL_StatusTypeDef Wait_For_I2C_Complete(uint32_t timeout_ms) {
    uint32_t start = get_millis();
    while (imu_i2c_state == I2C_BUSY) { // Az enum állapotodat használjuk
        if (get_millis() - start > timeout_ms) {
            debug_print("I2C Timeout!\r\n");
            return HAL_TIMEOUT;
        }
    }
    return HAL_OK;
}

// Skaláris szorzat (Dot Product)
// Megadja a két vektor által bezárt szög koszinuszát (ha egységvektorok)
float vector3_dot(Vector3 a, Vector3 b) {
    return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
}

// Keresztszorzat (Cross Product)
// Egy olyan vektort ad vissza, amely merőleges mindkét bemeneti vektorra
Vector3 vector3_cross(Vector3 a, Vector3 b) {
    Vector3 result;
    result.x = (a.y * b.z) - (a.z * b.y);
    result.y = (a.z * b.x) - (a.x * b.z);
    result.z = (a.x * b.y) - (a.y * b.x);
    return result;
}

// Normalizálás (Normalize)
// Egységnyi hosszúságúvá teszi a vektort (hossza = 1.0)
Vector3 vector3_normalize(Vector3 v) {
    float magSq = (v.x * v.x) + (v.y * v.y) + (v.z * v.z);

    if (magSq > 0.0000000001f) { // Biztonságos küszöb
        float invMag = 1.0f / sqrtf(magSq);
        return (Vector3){v.x * invMag, v.y * invMag, v.z * invMag};
    }

    return (Vector3){0.0f, 0.0f, 0.0f}; // Ha nulla a vektor hossza
}
float vector3_length(Vector3 v) {
    return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}
quaternion quaternion_multiply(quaternion q, quaternion r) {
    quaternion res;
    // X komponens
    res.x = q.w * r.x + q.x * r.w + q.y * r.z - q.z * r.y;
    // Y komponens
    res.y = q.w * r.y - q.x * r.z + q.y * r.w + q.z * r.x;
    // Z komponens
    res.z = q.w * r.z + q.x * r.y - q.y * r.x + q.z * r.w;
    // W komponens
    res.w = q.w * r.w - q.x * r.x - q.y * r.y - q.z * r.z;
    return res;
}

quaternion quaternion_normalize(quaternion q) {
    float magSq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;

    if (magSq > 0.000001f) {
        float invMag = 1.0f / sqrtf(magSq);
        return (quaternion){
            .x = q.x * invMag,
            .y = q.y * invMag,
            .z = q.z * invMag,
            .w = q.w * invMag
        };
    }
    return (quaternion){0.0f, 0.0f, 0.0f, 1.0f}; // Identity (Unity-ben 0,0,0,1)
}

euler_float quat_to_euler(quaternion q) {
    euler_float euler;

    // --- ROLL (X-tengely) ---
    // Az orr-farr tengely körüli elmozdulás
    float t0 = 2.0f * (q.w * q.x + q.y * q.z);
    float t1 = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    euler.roll = atan2f(t0, t1) * M_RAD2DEG;

    // --- PITCH (Y-tengely) ---
    // A szárny/kar tengely körüli elmozdulás (bólintás)
    // t2 kiszámítása a standard képlet alapján: 2.0 * (w*y - z*x)
    float t2 = 2.0f * (q.w * q.y - q.z * q.x);

    // Numerikus stabilitás: ha a gép függőlegesen áll (90 fok), az asinf NaN-t adna
    if (t2 > 1.0f) t2 = 1.0f;
    if (t2 < -1.0f) t2 = -1.0f;
    euler.pitch = asinf(t2) * M_RAD2DEG;

    // --- YAW (Z-tengely) ---
    // Függőleges tengely körüli elmozdulás (elfordulás)
    float t3 = 2.0f * (q.w * q.z + q.x * q.y);
    float t4 = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    euler.yaw = atan2f(t3, t4) * M_RAD2DEG;

    return euler;
}
