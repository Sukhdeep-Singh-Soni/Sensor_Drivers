################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/ICM20948/icm20948.c 

OBJS += \
./Drivers/ICM20948/icm20948.o 

C_DEPS += \
./Drivers/ICM20948/icm20948.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/ICM20948/%.o Drivers/ICM20948/%.su: ../Drivers/ICM20948/%.c Drivers/ICM20948/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F769xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I"C:/Projects/orientation_on_discovery_board/Drivers/bsp" -I"C:/Projects/orientation_on_discovery_board/Drivers/ICM20948" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-ICM20948

clean-Drivers-2f-ICM20948:
	-$(RM) ./Drivers/ICM20948/icm20948.d ./Drivers/ICM20948/icm20948.o ./Drivers/ICM20948/icm20948.su

.PHONY: clean-Drivers-2f-ICM20948

