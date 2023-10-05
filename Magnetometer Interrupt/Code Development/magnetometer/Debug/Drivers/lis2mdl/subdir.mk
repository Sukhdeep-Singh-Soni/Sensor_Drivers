################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/lis2mdl/lis2mdl.c \
../Drivers/lis2mdl/lis2mdl_reg.c 

OBJS += \
./Drivers/lis2mdl/lis2mdl.o \
./Drivers/lis2mdl/lis2mdl_reg.o 

C_DEPS += \
./Drivers/lis2mdl/lis2mdl.d \
./Drivers/lis2mdl/lis2mdl_reg.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/lis2mdl/%.o Drivers/lis2mdl/%.su: ../Drivers/lis2mdl/%.c Drivers/lis2mdl/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U585xx -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -I"C:/Projects/magnetometer/Drivers/BSP" -I"C:/Projects/magnetometer/Drivers/lis2mdl" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-lis2mdl

clean-Drivers-2f-lis2mdl:
	-$(RM) ./Drivers/lis2mdl/lis2mdl.d ./Drivers/lis2mdl/lis2mdl.o ./Drivers/lis2mdl/lis2mdl.su ./Drivers/lis2mdl/lis2mdl_reg.d ./Drivers/lis2mdl/lis2mdl_reg.o ./Drivers/lis2mdl/lis2mdl_reg.su

.PHONY: clean-Drivers-2f-lis2mdl

