################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/TOF/tof.c \
../Drivers/TOF/tof_reg.c 

OBJS += \
./Drivers/TOF/tof.o \
./Drivers/TOF/tof_reg.o 

C_DEPS += \
./Drivers/TOF/tof.d \
./Drivers/TOF/tof_reg.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/TOF/%.o Drivers/TOF/%.su: ../Drivers/TOF/%.c Drivers/TOF/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F767xx -c -I../Core/Inc -I"/home/sukhdeep/STM32CubeIDE/workspace_1.10.1/tof_gesture_detection_and_people_counting/Drivers/TOF" -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-TOF

clean-Drivers-2f-TOF:
	-$(RM) ./Drivers/TOF/tof.d ./Drivers/TOF/tof.o ./Drivers/TOF/tof.su ./Drivers/TOF/tof_reg.d ./Drivers/TOF/tof_reg.o ./Drivers/TOF/tof_reg.su

.PHONY: clean-Drivers-2f-TOF

