################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/lsm6dsox/lsm6dsox.c \
../Drivers/lsm6dsox/lsm6dsox_reg.c 

OBJS += \
./Drivers/lsm6dsox/lsm6dsox.o \
./Drivers/lsm6dsox/lsm6dsox_reg.o 

C_DEPS += \
./Drivers/lsm6dsox/lsm6dsox.d \
./Drivers/lsm6dsox/lsm6dsox_reg.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/lsm6dsox/%.o Drivers/lsm6dsox/%.su: ../Drivers/lsm6dsox/%.c Drivers/lsm6dsox/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U585xx -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -I"C:/Projects/motion_sensor/Drivers/BSP" -I"C:/Projects/motion_sensor/Drivers/lis2mdl" -I"C:/Projects/motion_sensor/Drivers/lsm6dsox" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-lsm6dsox

clean-Drivers-2f-lsm6dsox:
	-$(RM) ./Drivers/lsm6dsox/lsm6dsox.d ./Drivers/lsm6dsox/lsm6dsox.o ./Drivers/lsm6dsox/lsm6dsox.su ./Drivers/lsm6dsox/lsm6dsox_reg.d ./Drivers/lsm6dsox/lsm6dsox_reg.o ./Drivers/lsm6dsox/lsm6dsox_reg.su

.PHONY: clean-Drivers-2f-lsm6dsox

