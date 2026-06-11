/*
 * timers.h
 *
 *  Created on: 2026. máj. 9.
 *      Author: balin
 */

#ifndef INC_TIMERS_H_
#define INC_TIMERS_H_

#define INV_CPU_FREQ 			(1.0f / 64000000.0f)
#define CPU_HZ 					64000000UL
#define LOOP0_5_CYCLES			(CPU_HZ / 2000)   // 2 kHz
#define LOOP1_CYCLES			(CPU_HZ / 1000)   // 1 kHz
#define IMU_CYCLES				(CPU_HZ / 1000)
#define LOOP2_CYCLES			(CPU_HZ / 1000)
#define TOF_CYCLES				(CPU_HZ / 5)
#define OPTICAL_CYCLES			(CPU_HZ / 200)
#define LOOP10MS_CYCLES			(CPU_HZ / 100)
#define LOOP50MS_CYCLES			(CPU_HZ / 50)
#define LOOP100MS_CYCLES		(CPU_HZ / 10)
#define LOOP1S_CYCLES			(CPU_HZ)

extern volatile float imu_deltatime;
extern volatile float tof_deltatime;
extern volatile float pid_deltatime;
extern volatile uint32_t pid_lasttime;
extern volatile uint32_t imu_lasttime;
extern volatile uint32_t tof_lasttime;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;

void timers_init();
void delay_ms(uint32_t ms);
void delay_us(uint32_t us);

static inline uint32_t get_us(void) {
	 return DWT->CYCCNT;
}

static inline uint32_t pid_deltatime_us(void) {
    uint32_t now = DWT->CYCCNT;
    uint32_t dt = now - pid_lasttime;
    pid_lasttime = now;
    return dt;
}

#endif /* INC_TIMERS_H_ */
