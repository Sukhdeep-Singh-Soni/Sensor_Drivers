################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Touch_Sensor/at42qt2120.c 

C_DEPS += \
./Drivers/Touch_Sensor/at42qt2120.d 

OBJS += \
./Drivers/Touch_Sensor/at42qt2120.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Touch_Sensor/%.o Drivers/Touch_Sensor/%.su: ../Drivers/Touch_Sensor/%.c Drivers/Touch_Sensor/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F769xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I"D:/Code_Files/ATQT_Disco/QT2120" -I"D:/Code_Files/ATQT_Disco/Drivers/Touch_Sensor" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Touch_Sensor

clean-Drivers-2f-Touch_Sensor:
	-$(RM) ./Drivers/Touch_Sensor/at42qt2120.d ./Drivers/Touch_Sensor/at42qt2120.o ./Drivers/Touch_Sensor/at42qt2120.su

.PHONY: clean-Drivers-2f-Touch_Sensor

