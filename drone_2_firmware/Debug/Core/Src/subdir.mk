################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/COMM.c \
../Core/Src/IMU.c \
../Core/Src/MOTOR.c \
../Core/Src/Optic.c \
../Core/Src/PID_CONTROL.c \
../Core/Src/TOF.c \
../Core/Src/VL53L1X_api.c \
../Core/Src/VL53L1X_calibration.c \
../Core/Src/ekf.c \
../Core/Src/flight.c \
../Core/Src/main.c \
../Core/Src/stm32f3xx_hal_msp.c \
../Core/Src/stm32f3xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f3xx.c \
../Core/Src/timers.c \
../Core/Src/vl53l1_platform.c 

OBJS += \
./Core/Src/COMM.o \
./Core/Src/IMU.o \
./Core/Src/MOTOR.o \
./Core/Src/Optic.o \
./Core/Src/PID_CONTROL.o \
./Core/Src/TOF.o \
./Core/Src/VL53L1X_api.o \
./Core/Src/VL53L1X_calibration.o \
./Core/Src/ekf.o \
./Core/Src/flight.o \
./Core/Src/main.o \
./Core/Src/stm32f3xx_hal_msp.o \
./Core/Src/stm32f3xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f3xx.o \
./Core/Src/timers.o \
./Core/Src/vl53l1_platform.o 

C_DEPS += \
./Core/Src/COMM.d \
./Core/Src/IMU.d \
./Core/Src/MOTOR.d \
./Core/Src/Optic.d \
./Core/Src/PID_CONTROL.d \
./Core/Src/TOF.d \
./Core/Src/VL53L1X_api.d \
./Core/Src/VL53L1X_calibration.d \
./Core/Src/ekf.d \
./Core/Src/flight.d \
./Core/Src/main.d \
./Core/Src/stm32f3xx_hal_msp.d \
./Core/Src/stm32f3xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f3xx.d \
./Core/Src/timers.d \
./Core/Src/vl53l1_platform.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303xC -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/COMM.cyclo ./Core/Src/COMM.d ./Core/Src/COMM.o ./Core/Src/COMM.su ./Core/Src/IMU.cyclo ./Core/Src/IMU.d ./Core/Src/IMU.o ./Core/Src/IMU.su ./Core/Src/MOTOR.cyclo ./Core/Src/MOTOR.d ./Core/Src/MOTOR.o ./Core/Src/MOTOR.su ./Core/Src/Optic.cyclo ./Core/Src/Optic.d ./Core/Src/Optic.o ./Core/Src/Optic.su ./Core/Src/PID_CONTROL.cyclo ./Core/Src/PID_CONTROL.d ./Core/Src/PID_CONTROL.o ./Core/Src/PID_CONTROL.su ./Core/Src/TOF.cyclo ./Core/Src/TOF.d ./Core/Src/TOF.o ./Core/Src/TOF.su ./Core/Src/VL53L1X_api.cyclo ./Core/Src/VL53L1X_api.d ./Core/Src/VL53L1X_api.o ./Core/Src/VL53L1X_api.su ./Core/Src/VL53L1X_calibration.cyclo ./Core/Src/VL53L1X_calibration.d ./Core/Src/VL53L1X_calibration.o ./Core/Src/VL53L1X_calibration.su ./Core/Src/ekf.cyclo ./Core/Src/ekf.d ./Core/Src/ekf.o ./Core/Src/ekf.su ./Core/Src/flight.cyclo ./Core/Src/flight.d ./Core/Src/flight.o ./Core/Src/flight.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32f3xx_hal_msp.cyclo ./Core/Src/stm32f3xx_hal_msp.d ./Core/Src/stm32f3xx_hal_msp.o ./Core/Src/stm32f3xx_hal_msp.su ./Core/Src/stm32f3xx_it.cyclo ./Core/Src/stm32f3xx_it.d ./Core/Src/stm32f3xx_it.o ./Core/Src/stm32f3xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f3xx.cyclo ./Core/Src/system_stm32f3xx.d ./Core/Src/system_stm32f3xx.o ./Core/Src/system_stm32f3xx.su ./Core/Src/timers.cyclo ./Core/Src/timers.d ./Core/Src/timers.o ./Core/Src/timers.su ./Core/Src/vl53l1_platform.cyclo ./Core/Src/vl53l1_platform.d ./Core/Src/vl53l1_platform.o ./Core/Src/vl53l1_platform.su

.PHONY: clean-Core-2f-Src

