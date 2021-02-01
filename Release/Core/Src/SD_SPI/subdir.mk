################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/SD_SPI/fatfs_sd.c \
../Core/Src/SD_SPI/sd_spi.c 

OBJS += \
./Core/Src/SD_SPI/fatfs_sd.o \
./Core/Src/SD_SPI/sd_spi.o 

C_DEPS += \
./Core/Src/SD_SPI/fatfs_sd.d \
./Core/Src/SD_SPI/sd_spi.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/SD_SPI/fatfs_sd.o: ../Core/Src/SD_SPI/fatfs_sd.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F303xC -c -I../FATFS/App -I../Drivers/CMSIS/Include -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../FATFS/Target -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Middlewares/Third_Party/FatFs/src -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/SD_SPI/fatfs_sd.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/SD_SPI/sd_spi.o: ../Core/Src/SD_SPI/sd_spi.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F303xC -c -I../FATFS/App -I../Drivers/CMSIS/Include -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../FATFS/Target -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Middlewares/Third_Party/FatFs/src -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/SD_SPI/sd_spi.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

