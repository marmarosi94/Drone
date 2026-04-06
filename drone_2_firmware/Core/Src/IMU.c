/*
 * IMU.c
 *
 *  Created on: Apr 1, 2026
 *      Author: balin
 */
#include "IMU.h"


#define IMU_I2C_ADDRESS 0xD0 // Example I2C address of the IMU (MPU6050) - 7-bit: 0x68
// Registers for configuration
#define PWR_MGMT_1_REG  0x6B
#define CONFIG_REG      0x1A
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define SMPLRT_DIV_REG  0x19

// ==== GLOBAL VARIABLE DEFINITIONS ====
int bias_sample_cnt = 0;
uint8_t gyro_is_calibrated = 0;
uint8_t imu_data[14];

// Accelerometer
vec3_q12_t accel = {0};

Vector3 gyro_Bias = {0};
Vector3 gyro_Sample = {0};
vec3_q12_t gyro_Bias_q12 = {0};
vec3_q12_t gyro_meas = {0};
vec3_q12_t gyro_delta = {0};
Vector3 gyro;
q12_t gyro_angle;
vec3_q12_t gyro_to_rad;

vec3_q12_t g_ref = {0,0,Q12_ONE};
vec3_q12_t g_meas = {0,0,0};
vec3_q12_t acc_Deg = {0};

quat_q12_t quat_acc   = {Q12_ONE,0,0,0};
quat_q12_t quat_gyro = {Q12_ONE,0,0,0};
quat_q12_t quat_delta = {Q12_ONE,0,0,0};
quat_q12_t quat_flt_orientation = {0};

vec3_q12_t error_vector = {0};
vec3_q12_t error_int = {0};

// Function to initialize the IMU
void IMU_Init(void) {
    uint8_t data[2];
    char str[64];
    // Wake up MPU6050 (write 0x00 to PWR_MGMT_1 register)
    data[0] = 0x00;   // Value to wake up the IMU
    if (I2C_Write_DMA(IMU_I2C_ADDRESS + 1, 0x6B, data, 1) != HAL_OK) {
        debug_print("Error: Failed to wake up IMU\r\n");
        return;  // Exit if I2C write failed
    }

    delay(100);  // Need 100ms after the reset for the IMU

    uint8_t i = 0;
    uint32_t start_time = get_millis();
    uint32_t timeout_duration = 1000;  // Timeout after 1000 ms (1 second)
    uint32_t retry_delay = 50;  // Delay between retry attempts (in ms)

    // Loop through retry attempts for IMU detection, prioritizing retries
    while (i < 10) {  // Max 10 attempts, retry-focused
        // Clear the DMA completion flag before starting a new read
        dma_read_complete = 0;

        // Start I2C read with DMA (non-blocking call)
        I2C_Read_DMA(IMU_I2C_ADDRESS, 0x75, data, 1);

        // Wait for DMA read to complete with timeout logic
        uint32_t wait_start_time = get_millis();
        while (!dma_read_complete) {
            // If the overall timeout period has passed, exit with an error
            if (get_millis() - start_time > timeout_duration) {
                debug_print("Error: IMU not responding within timeout period\r\n");
                return;  // Exit if overall timeout has been exceeded
            }

            // If retry delay threshold has passed, break the inner loop and retry
            if (get_millis() - wait_start_time > retry_delay) {
                break;  // Exit the current wait loop and try again
            }
        }

        // Check if the IMU is responding correctly (should return 0x68)
        if (data[0] == 0x68) {
            debug_print("IMU FOUND!\r\n");
            IMU_Config_Fast_Mode();
            return;  // Successfully found the IMU, exit the function
        } else {
            sprintf(str, "IMU NOT found, attempt %d\r\n", i);
            debug_print(str);  // Print attempt message
        }

        // Increment retry attempt
        i++;

        // Add a small delay before retrying (to prevent hammering)
        HAL_Delay(10);  // 10 ms delay between retries (can be adjusted)
    }

    // If we exit the loop without finding the IMU, handle the timeout
    debug_print("Error: IMU search exceeded maximum attempts\r\n");
}

// Function to configure the MPU-6050 for the fastest mode with the low-pass filter
// Function to configure the MPU-6050 for the fastest mode with an optional low-pass filter
void IMU_Config_Fast_Mode(void) {
    uint8_t data[2];

    // Wait for DMA transfer to complete with timeout logic
    uint32_t wait_start_time = get_millis();
    uint32_t timeout_duration = 1000;  // Timeout duration for reading (in ms)


    // Step 2: Configure the low-pass filter (DLPF)
    // Optional: Set the DLPF to a moderate speed (256Hz cutoff, if you prefer smoothing)
    data[0] = 0x00;  // DLPF_CFG = 0x01 (256Hz cutoff filter)
    I2C_Write_DMA(IMU_I2C_ADDRESS, CONFIG_REG, data, 1);
    // Wait for DMA transfer to complete or timeout
    while (!dma_write_complete) {
        if (get_millis() - wait_start_time > timeout_duration) {
            debug_print("Error: DMA read timeout\r\n");
            //return;  // Exit if the read operation times out
        }
    }
    // Start I2C read with DMA for accelerometer and gyroscope data (14 bytes)
    I2C_Read_DMA(IMU_I2C_ADDRESS, CONFIG_REG, &data[1], 1);
    // Wait for DMA transfer to complete or timeout
    while (!dma_read_complete) {
        if (get_millis() - wait_start_time > timeout_duration) {
            debug_print("Error: DMA read timeout\r\n");
            //return;  // Exit if the read operation times out
        }
    }

    // Step 3: Configure accelerometer range to ±16g
    data[0] = 0x18;
    I2C_Write_DMA(IMU_I2C_ADDRESS, ACCEL_CONFIG_REG, data, 1);
    // Wait for DMA transfer to complete or timeout
    while (!dma_write_complete) {
        if (get_millis() - wait_start_time > timeout_duration) {
            debug_print("Error: DMA read timeout\r\n");
            //return;  // Exit if the read operation times out
        }
    }
    I2C_Read_DMA(IMU_I2C_ADDRESS, ACCEL_CONFIG_REG, &data[1], 1);
    // Wait for DMA transfer to complete or timeout
    while (!dma_read_complete) {
        if (get_millis() - wait_start_time > timeout_duration) {
            debug_print("Error: DMA read timeout\r\n");
            //return;  // Exit if the read operation times out
        }
    }

    // Step 4: Configure gyroscope range to ±2000°/s
    data[0] = 0x18;  // FS_SEL = 0x01 (±2000°/s range for gyroscope)
    I2C_Write_DMA(IMU_I2C_ADDRESS, GYRO_CONFIG_REG, data, 1);
    // Wait for DMA transfer to complete or timeout
    while (!dma_write_complete) {
        if (get_millis() - wait_start_time > timeout_duration) {
            debug_print("Error: DMA read timeout\r\n");
            //return;  // Exit if the read operation times out
        }
    }
    I2C_Read_DMA(IMU_I2C_ADDRESS, GYRO_CONFIG_REG, &data[1], 1);
    // Wait for DMA transfer to complete or timeout
    while (!dma_read_complete) {
        if (get_millis() - wait_start_time > timeout_duration) {
            debug_print("Error: DMA read timeout\r\n");
            //return;  // Exit if the read operation times out
        }
    }

    // Step 5: Set sampling rate to the (SMPLRT_DIV = 8)
    data[0] = 0x08;  // SMPLRT_DIV = 8 for ampling rate (1kHz)
    I2C_Write_DMA(IMU_I2C_ADDRESS, SMPLRT_DIV_REG, data, 1);
    // Wait for DMA transfer to complete or timeout
    while (!dma_write_complete) {
        if (get_millis() - wait_start_time > timeout_duration) {
            debug_print("Error: DMA read timeout\r\n");
            //return;  // Exit if the read operation times out
        }
    }
    I2C_Read_DMA(IMU_I2C_ADDRESS, SMPLRT_DIV_REG, &data[1], 1);
    // Wait for DMA transfer to complete or timeout
    while (!dma_read_complete) {
        if (get_millis() - wait_start_time > timeout_duration) {
            debug_print("Error: DMA read timeout\r\n");
            //return;  // Exit if the read operation times out
        }
    }

    // Print confirmation
    debug_print("MPU-6050 configured for fastest mode!\r\n");
}


// Function to read accelerometer and gyroscope data
void IMU_Read_Accel_Gyro(void) {


    // Wait for DMA transfer to complete with timeout logic
    uint32_t wait_start_time = get_millis();
    uint32_t timeout_duration = 1000;  // Timeout duration for reading (in ms)

    // Start I2C read with DMA for accelerometer and gyroscope data (14 bytes)
    I2C_Read_DMA(IMU_I2C_ADDRESS, 0x3B, imu_data, 14);


    // Wait for DMA transfer to complete or timeout
    while (!dma_read_complete) {
        if (get_millis() - wait_start_time > timeout_duration) {
            debug_print("Error: DMA read timeout\r\n");
            return;  // Exit if the read operation times out
        }
    }

    // Process accelerometer data
    accel.x = (int16_t)((imu_data[0] << 8) | imu_data[1]);
    accel.y = (int16_t)((imu_data[2] << 8) | imu_data[3]);
    accel.z = (int16_t)((imu_data[4] << 8) | imu_data[5]);

    // Process gyroscope data
    gyro.x = (int16_t)((imu_data[8] << 8) | imu_data[9]);
    gyro.y = (int16_t)((imu_data[10] << 8) | imu_data[11]);
    gyro.z = (int16_t)((imu_data[12] << 8) | imu_data[13]);
}

void IMU_compute_rotation(){
	char str[256];

    g_meas.x = q12_div(q12_from_int(accel.x), q12_from_int(ACC_LSB));
    g_meas.y = q12_div(q12_from_int(accel.y), q12_from_int(ACC_LSB));
    g_meas.z = q12_div(q12_from_int(accel.z), q12_from_int(ACC_LSB));

    // Normalize measurement
    g_meas = vec3_normalize(g_meas);

    // Compute quaternion directly
    vec3_q12_t cross = vec3_cross(g_ref, g_meas);
    q12_t dot = vec3_dot(g_ref, g_meas);

    quat_acc.x = cross.y;
    quat_acc.y = cross.z;
    quat_acc.z = -cross.x;
    quat_acc.w = 4096 + dot;

    // Normalize quaternion
    quat_acc = quat_normalize(quat_acc);
	//sprintf(str, "$%i,%i,%i,%i\r\n",quat_acc.x, quat_acc.y, quat_acc.z, quat_acc.w);


    //AVG for gyro drift compestion
    if(gyro_is_calibrated == 0)
    {
    	gyro_Sample.x = gyro_Sample.x + gyro.x;          //Summary for avarege of gyro datas
    	gyro_Sample.y = gyro_Sample.y + gyro.y;             //Summary for avarege of gyro datas
    	gyro_Sample.z = gyro_Sample.z + gyro.z;                //Summary for avarege of gyro datas
    	sprintf(str, "gyro_sample:%i,%i,%i\r\n",gyro_Sample.x, gyro_Sample.y, gyro_Sample.z);
    	debug_print(str);  // Send gyroscope data to PC
    	bias_sample_cnt++;
        if (BIAS_CALIB_SAMPLE_QTY <= bias_sample_cnt)
        {
        	gyro_is_calibrated = 1;
        }
    }
    if(gyro_is_calibrated == 1)
	{
		// Calculate high-precision Bias directly into Q12
		gyro_Bias_q12.x = (gyro_Sample.x << Q12_SHIFT) / bias_sample_cnt;
		gyro_Bias_q12.y = (gyro_Sample.y << Q12_SHIFT) / bias_sample_cnt;
		gyro_Bias_q12.z = (gyro_Sample.z << Q12_SHIFT) / bias_sample_cnt;

		sprintf(str, "Q12 Bias:%ld, %ld, %ld\r\n", gyro_Bias_q12.x, gyro_Bias_q12.y, gyro_Bias_q12.z);
		debug_print(str);

		// Take first values for the integration from the accelerometer
		quat_gyro = quat_acc;

		currenttime = get_millis();
		gyro_is_calibrated = 2;
	}
    if(gyro_is_calibrated == 2)
	{
		// 1. Get raw values in Q12 and subtract the high-precision Q12 bias
		gyro_meas.x = -(q12_from_int(gyro.y) - gyro_Bias_q12.y);
		gyro_meas.y = -(q12_from_int(gyro.z) - gyro_Bias_q12.z);
		gyro_meas.z =  (q12_from_int(gyro.x) - gyro_Bias_q12.x);

		// dt is returned in Q12 (e.g., 10ms = ~40)
		deltatime = get_deltatime();

		// 2. Multiply rate by time (Q12 * Q12 = Q12 using your safe function)
		gyro_delta.x = q12_mul(gyro_meas.x, deltatime);
		gyro_delta.y = q12_mul(gyro_meas.y, deltatime);
		gyro_delta.z = q12_mul(gyro_meas.z, deltatime);

		// 3. Scale to radians and divide by 2 for the quaternion derivative
		// GYRO_SCALE is 0.001065 rad/s/LSB. Half of that is 0.0005325.
		// To do this in pure integer math without losing data:
		// 0.0005325 * 1048576 (which is 2^20) = 558.
		// We multiply by 558, then shift right by 20 to return perfectly to Q12 space.
		quat_delta.w = Q12_ONE;
		quat_delta.x = ((int64_t)gyro_delta.x * 558) >> 20;
		quat_delta.y = ((int64_t)gyro_delta.y * 558) >> 20;
		quat_delta.z = ((int64_t)gyro_delta.z * 558) >> 20;

		// 4. Integrate and Normalize
		quat_gyro = quat_mul(quat_gyro, quat_delta);
		quat_gyro = quat_normalize(quat_gyro);

		// 5. Fixed-Point Complementary Filter (98% Gyro, 2% Accel)
		// Multiply by integers, divide by 100. Fast and float-free!
		quat_gyro.w = (quat_gyro.w * 98 + quat_acc.w * 2) / 100;
		quat_gyro.x = (quat_gyro.x * 98 + quat_acc.x * 2) / 100;
		quat_gyro.y = (quat_gyro.y * 100 + quat_acc.y * 0) / 100;
		quat_gyro.z = (quat_gyro.z * 98 + quat_acc.z * 2) / 100;

		// Normalize the final result and FEED IT BACK into quat_gyro
		quat_gyro = quat_normalize(quat_gyro);

		// Copy to your output variable
		quat_flt_orientation = quat_gyro;

		sprintf(str, "$%ld,%ld,%ld,%ld\r\n", quat_flt_orientation.x, quat_flt_orientation.y, quat_flt_orientation.z, quat_flt_orientation.w);
		debug_print(str);
	}
}

