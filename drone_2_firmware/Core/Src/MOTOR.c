/*
 * MOTOR.c
 *
 *  Created on: Apr 9, 2026
 *      Author: balin
 */

#include "MOTOR.h"


void esc_init()
{
    // 1. Timerek fizikai indítása
    HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1); 	// Motor 1 (PA12) - OK
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); 	// Motor 2 (PA6) - OK
    HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1); 	// Motor 3 (PA7) - OK
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); 	// Motor 4 (PB11) - Ok
    __HAL_TIM_MOE_ENABLE(&htim1);
    __HAL_TIM_MOE_ENABLE(&htim17);
    HAL_Delay(1000);
    // 3. Biztonsági indítás: 1000us jellel inicializáljuk az ESC-ket
    set_all_motors(1000);
    HAL_Delay(1000);
}
/**
 * @brief Beállítja mind a négy motor sebességét (1000-2000 mikroszekundum)
 * @param throttle: 1000 (leállítva) és 2000 (teljes gáz) közötti érték
 */
void set_all_motors(int16_t throttle) {
    if (throttle < 1000) throttle = 1000;
    if (throttle > 2000) throttle = 2000;

    // TIM3 Csatornák
    __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, throttle); 	// Motor 1 (PA12)
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, throttle); 	// Motor 2 (PA6)
    __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, throttle); 	// Motor 3 (PA7)
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, throttle); 	// Motor 4 (PB11)
}

void set_motor1_speed(int16_t throttle){
    if (throttle < MIN_THROTTLE) throttle = MIN_THROTTLE;
    if (throttle > MAX_THROTTLE) throttle = MAX_THROTTLE;

    __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, throttle); // Motor 1 (PA12)
}

void set_motor2_speed(int16_t throttle){
    if (throttle < MIN_THROTTLE) throttle = MIN_THROTTLE;
    if (throttle > MAX_THROTTLE) throttle = MAX_THROTTLE;

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, throttle); 	// Motor 2 (PA6)
}

void set_motor3_speed(int16_t throttle){
    if (throttle < MIN_THROTTLE) throttle = MIN_THROTTLE;
    if (throttle > MAX_THROTTLE) throttle = MAX_THROTTLE;

    __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, throttle); 	// Motor 3 (PA7)
}

void set_motor4_speed(int16_t throttle){
    if (throttle < MIN_THROTTLE) throttle = MIN_THROTTLE;
    if (throttle > MAX_THROTTLE) throttle = MAX_THROTTLE;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, throttle); 	// Motor 4 (PB11)
}
