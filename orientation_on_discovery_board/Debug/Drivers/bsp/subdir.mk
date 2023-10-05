################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/bsp/orientation_bsp.c 

OBJS += \
./Drivers/bsp/orientation_bsp.o 

C_DEPS += \
./Drivers/bsp/orientation_bsp.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/bsp/%.o Drivers/bsp/%.su: ../Drivers/bsp/%.c Drivers/bsp/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F769xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I"C:/Projects/orientation_on_discovery_board/Drivers/bsp" -I"C:/Projects/orientation_on_discovery_board/Drivers/ICM20948" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-bsp

clean-Drivers-2f-bsp:
	-$(RM) ./Drivers/bsp/orientation_bsp.d ./Drivers/bsp/orientation_bsp.o ./Drivers/bsp/orientation_bsp.su

.PHONY: clean-Drivers-2f-bsp

