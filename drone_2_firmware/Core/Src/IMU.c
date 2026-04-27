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
uint8_t imu_data_is_new = 0;
uint8_t  t_calib = 0;

// Accelerometer
Vector3 accel = {0};
Vector3 accel_prev = {0};
Vector3 gyro_Bias = {0};
Vector3 gyro_Sample = {0};
Vector3 gyro_frame = {0};
Vector3 gyro_frame_deg = {0};
Vector3 gyro;
Vector3 gyro_prev = {0};
Vector3 g_ref = {0,0,1};
Vector3 gravity_meas = {0,0,0};
Vector3 gyro_bias_integral = {0, 0, 0}; // Az integrált hiba tárolója
quaternion quat_gyro = {1,0,0,0};
quaternion quat_delta = {1,0,0,0};
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

    // Sample Rate Divider (Regiszter 0x19)
    // 0x00 = 1kHz belső mintavételezés
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, i2c_addr, 0x19, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);

    // DLPF (Low Pass Filter) beállítása (Regiszter 0x1A)
    // 0x03 = ~42Hz cutoff (kiszűri a motorvibrációt)
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

    // Regiszterek és elvárt értékeik listája
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
void IMU_Request_Data(void){
    if(dma_read_complete == I2C_IDLE) {
        I2C_Read_DMA(IMU_I2C_ADDRESS, 0x3A, (uint8_t *)&imu_raw, 15);
    }
}
// Function to read accelerometer and gyroscope data
void IMU_Parse_Data(void){
    if(dma_read_complete == I2C_IDLE) {
        I2C_Read_DMA(IMU_I2C_ADDRESS, 0x3A, (uint8_t *)&imu_raw, 15);
    }
    if(dma_read_complete == I2C_COMPLETE) {
			// Note: imu_data[6-7] is Temperature, which is why we skip to [8]
			accel.x = (int16_t)((imu_raw.imu_data[0] << 8) | imu_raw.imu_data[1]);
			accel.y = (int16_t)((imu_raw.imu_data[2] << 8) | imu_raw.imu_data[3]);
			accel.z = (int16_t)((imu_raw.imu_data[4] << 8) | imu_raw.imu_data[5]);

			gyro.x  = (int16_t)((imu_raw.imu_data[8] << 8) | imu_raw.imu_data[9]);
			gyro.y  = (int16_t)((imu_raw.imu_data[10] << 8) | imu_raw.imu_data[11]);
			gyro.z  = (int16_t)((imu_raw.imu_data[12] << 8) | imu_raw.imu_data[13]);
			dma_read_complete = I2C_IDLE;
    }
}

void IMU_Calib(){
	while(1){
		  uint32_t now = get_us();
		  if ((uint32_t)(now - t_calib) >= LOOP1_US) {
		      if(dma_read_complete == I2C_COMPLETE) {
		          if(imu_raw.status & 0x01) {
		              IMU_Parse_Data();         // Extract the bytes safely

		              // GYRO BIAS TANÍTÁS (IMU térben OK) ---
		              if(gyro_is_calibrated == 0){
		                  gyro_Sample.x += gyro.x;
		                  gyro_Sample.y += gyro.y;
		                  gyro_Sample.z += gyro.z;
		                  bias_sample_cnt++;

		                  if (BIAS_CALIB_SAMPLE_QTY <= bias_sample_cnt)
		                      gyro_is_calibrated = 1;
		              }

		              if(gyro_is_calibrated == 1){
		                  gyro_Bias.x = (float) gyro_Sample.x / bias_sample_cnt;
		                  gyro_Bias.y = (float) gyro_Sample.y / bias_sample_cnt;
		                  gyro_Bias.z = (float) gyro_Sample.z / bias_sample_cnt;
		                  gyro_is_calibrated = 2;
		              }
		              if(gyro_is_calibrated == 2){
		            	  debug_print("Gyro calibrated!\r\n");
		            	  return;
		              }
		              imu_raw.status = 0;       // Clear status
		          }
		          dma_read_complete = I2C_IDLE;
		      }
		      IMU_Request_Data();
		      t_calib += LOOP1_US;
		  }
	}
}

void IMU_compute_rotation() {

    // --- 1. ACCEL: IMU -> VÁZ (-90° Z rotáció) ---
    float ax = (float)accel.x / ACC_LSB;
    float ay = (float)accel.y / ACC_LSB;
    float az = (float)accel.z / ACC_LSB;

    float ax_vaz =  ay;
    float ay_vaz = -ax;
    float az_vaz =  az;

    gravity_meas.x = ax_vaz;
    gravity_meas.y = ay_vaz;
    gravity_meas.z = az_vaz;
    gravity_meas = vector3_normalize(gravity_meas);

    // --- 2. GYRO: bias levonás + IMU -> VÁZ ---
    if(gyro_is_calibrated == 2) {

        float gx = ((float)gyro.x - gyro_Bias.x) * GYRO_SCALE;
        float gy = ((float)gyro.y - gyro_Bias.y) * GYRO_SCALE;
        float gz = ((float)gyro.z - gyro_Bias.z) * GYRO_SCALE;

        float gx_vaz =  gy;
        float gy_vaz = -gx;
        float gz_vaz =  gz;

        gyro_frame_deg.x = gx_vaz;
        gyro_frame_deg.y = gy_vaz;
        gyro_frame_deg.z = gz_vaz;

        gyro_frame.x = gx_vaz * DEG2RAD;
        gyro_frame.y = gy_vaz * DEG2RAD;
        gyro_frame.z = gz_vaz * DEG2RAD;



        // --- 3. MAHONY ---
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

        deltatime = imu_deltatime_us() * 1e-6f;

        gyro_bias_integral.x += error.x * KI * deltatime;
        gyro_bias_integral.y += error.y * KI * deltatime;
        gyro_bias_integral.z += error.z * KI * deltatime;

        gyro_frame.x += KP * error.x + gyro_bias_integral.x;
        gyro_frame.y += KP * error.y + gyro_bias_integral.y;
        gyro_frame.z += KP * error.z + gyro_bias_integral.z;


        // --- 4. KVATERNIÓ INTEGRÁLÁS ---
        float angle = vector3_length(gyro_frame) * deltatime;

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
}
void IMU_compute_position(){
    // --- HELYES: használd a valódi accel értéket ---
    Vector3 acc_body;
    acc_body.x = accel.x / ACC_LSB;
    acc_body.y = accel.y / ACC_LSB;
    acc_body.z = accel.z / ACC_LSB;

    // + alkalmazd ugyanazt a tengelyrotációt mint korábban
    float tmp;

    tmp = acc_body.x;
    acc_body.x =  acc_body.y;
    acc_body.y = -tmp;
    // z marad

    // --- 2. GRAVITÁCIÓ (quat-ból) ---
    Vector3 g_est;

    float qw = quat_flt_orientation.w;
    float qx = quat_flt_orientation.x;
    float qy = quat_flt_orientation.y;
    float qz = quat_flt_orientation.z;

    g_est.x = 2.0f * (qx*qz - qw*qy);
    g_est.y = 2.0f * (qw*qx + qy*qz);
    g_est.z = qw*qw - qx*qx - qy*qy + qz*qz;

    // --- 3. LINEÁRIS GYORSULÁS ---
    Vector3 lin_acc_body;

    lin_acc_body.x = acc_body.x - g_est.x;
    lin_acc_body.y = acc_body.y - g_est.y;
    lin_acc_body.z = acc_body.z - g_est.z;

    // --- 4. ROTÁCIÓ WORLD FRAME-BE ---
    Vector3 lin_acc_earth;

    Vector3 q_vec = {qx, qy, qz};
    Vector3 t = vector3_cross(q_vec, lin_acc_body);

    t.x *= 2.0f;
    t.y *= 2.0f;
    t.z *= 2.0f;

    lin_acc_earth.x = lin_acc_body.x + qw * t.x + (qy * t.z - qz * t.y);
    lin_acc_earth.y = lin_acc_body.y + qw * t.y + (qz * t.x - qx * t.z);
    lin_acc_earth.z = lin_acc_body.z + qw * t.z + (qx * t.y - qy * t.x);

    // --- 5. m/s^2 ---
    lin_acc_earth.x *= 9.81f;
    lin_acc_earth.y *= 9.81f;
    lin_acc_earth.z *= 9.81f;

    // --- 6. INTEGRÁLÁS (MINIMÁLIS SZŰRÉSSEL) ---
    float deadband = 0.1f;

    if (fabs(lin_acc_earth.x) < deadband) lin_acc_earth.x = 0;
    if (fabs(lin_acc_earth.y) < deadband) lin_acc_earth.y = 0;
    if (fabs(lin_acc_earth.z) < deadband) lin_acc_earth.z = 0;

    velocity.x += lin_acc_earth.x * deltatime;
    velocity.y += lin_acc_earth.y * deltatime;
    velocity.z += lin_acc_earth.z * deltatime;

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

    if (magSq > 0.000001f) { // Biztonságos küszöb
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
            .w = q.w * invMag, // Célszerű itt is a definíció sorrendjét tartani
            .x = q.x * invMag,
            .y = q.y * invMag,
            .z = q.z * invMag
        };
    }
    // Helyes Identitás inicializálás
    return (quaternion){ .w = 1.0f, .x = 0.0f, .y = 0.0f, .z = 0.0f };
}

euler_float quat_to_euler(quaternion q)
{
    euler_float euler;

    // feltételezzük: q normalizált

    float t0 = 2.0f * (q.w * q.x + q.y * q.z);
    float t1 = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    euler.roll = atan2f(t0, t1) * RAD2DEG;

    float t2 = 2.0f * (q.w * q.y - q.z * q.x);
    t2 = fmaxf(-1.0f, fminf(1.0f, t2));
    euler.pitch = asinf(t2) * RAD2DEG;

    float t3 = 2.0f * (q.w * q.z + q.x * q.y);
    float t4 = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    euler.yaw = atan2f(t3, t4) * RAD2DEG;

    return euler;
}
