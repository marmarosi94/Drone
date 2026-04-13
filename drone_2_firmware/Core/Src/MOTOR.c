/*
 * MOTOR.c
 *
 *  Created on: Apr 9, 2026
 *      Author: balin
 */

#include "MOTOR.h"


void esc_init()
{
    // 1. Kényszerítsünk minden PWM kimenetet 0-ra (Low) az elején
    // Ez biztosítja, hogy az ESC észlelje a jel megindulását
    __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim3,  TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1,  TIM_CHANNEL_4, 0);

    // 2. Indítsuk el a timereket 0-s kitöltéssel
    HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3,  TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1,  TIM_CHANNEL_4);

    __HAL_TIM_MOE_ENABLE(&htim1);
    __HAL_TIM_MOE_ENABLE(&htim16);
    __HAL_TIM_MOE_ENABLE(&htim17);

    // 3. Várjunk egy kicsit, hogy az ESC-k is bebootoljanak (tü-tü-tü vége)
    HAL_Delay(1000);

    // 4. Most adjuk ki a stabil 1000-et (vagy 1010-et a biztonság kedvéért)
    set_all_motors(1000);

    // 5. Adjunk időt az ESC-nek, hogy ráálljon a jelre és élesítsen
    HAL_Delay(2000);

    debug_print("ESC arming sequence complete.\r\n");
}
/**
 * @brief Beállítja mind a négy motor sebességét (1000-2000 mikroszekundum)
 * @param throttle: 1000 (leállítva) és 2000 (teljes gáz) közötti érték
 */
void set_all_motors(int16_t throttle) {
    if (throttle > 2000) throttle = 2000;
    if (throttle < 1000) throttle = 1000;
    __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, throttle); 	// Motor 1 (PA12)
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, throttle); 	// Motor 2 (PA6)
    __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, throttle); 	// Motor 3 (PA7)
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, throttle); 	// Motor 4 (PB11)
}

void set_motor1_speed(int16_t speed){
    if (speed < MIN_THROTTLE) speed = MIN_THROTTLE;
    if (speed > MAX_THROTTLE) speed = MAX_THROTTLE;

    __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, speed); // Motor 1 (PA12)
}

void set_motor2_speed(int16_t speed){
    if (speed < MIN_THROTTLE) speed = MIN_THROTTLE;
    if (speed > MAX_THROTTLE) speed = MAX_THROTTLE;

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed); 	// Motor 2 (PA6)
}

void set_motor3_speed(int16_t speed){
    if (speed < MIN_THROTTLE) speed = MIN_THROTTLE;
    if (speed > MAX_THROTTLE) speed = MAX_THROTTLE;

    __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, speed); 	// Motor 3 (PA7)
}

void set_motor4_speed(int16_t speed){
    if (speed < MIN_THROTTLE) speed = MIN_THROTTLE;
    if (speed > MAX_THROTTLE) speed = MAX_THROTTLE;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, speed); 	// Motor 4 (PB11)
}
