################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/MPU9250/MPU9250.c \
../Core/Src/MPU9250/Madgwick.c 

OBJS += \
./Core/Src/MPU9250/MPU9250.o \
./Core/Src/MPU9250/Madgwick.o 

C_DEPS += \
./Core/Src/MPU9250/MPU9250.d \
./Core/Src/MPU9250/Madgwick.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/MPU9250/MPU9250.o: ../Core/Src/MPU9250/MPU9250.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F303xC -c -I../FATFS/App -I../Drivers/CMSIS/Include -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../FATFS/Target -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Middlewares/Third_Party/FatFs/src -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/MPU9250/MPU9250.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/MPU9250/Madgwick.o: ../Core/Src/MPU9250/Madgwick.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F303xC -c -I../FATFS/App -I../Drivers/CMSIS/Include -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../FATFS/Target -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Middlewares/Third_Party/FatFs/src -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/MPU9250/Madgwick.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

