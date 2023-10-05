################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../QT2120/Appfruits_QTouch.cpp \
../QT2120/Arg_mvp_Qtouch.cpp \
../QT2120/TouchWheel.cpp 

OBJS += \
./QT2120/Appfruits_QTouch.o \
./QT2120/Arg_mvp_Qtouch.o \
./QT2120/TouchWheel.o 

CPP_DEPS += \
./QT2120/Appfruits_QTouch.d \
./QT2120/Arg_mvp_Qtouch.d \
./QT2120/TouchWheel.d 


# Each subdirectory must supply rules for building sources it contributes
QT2120/%.o QT2120/%.su: ../QT2120/%.cpp QT2120/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F769xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I"D:/Code_Files/ATQT_Disco/QT2120" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-QT2120

clean-QT2120:
	-$(RM) ./QT2120/Appfruits_QTouch.d ./QT2120/Appfruits_QTouch.o ./QT2120/Appfruits_QTouch.su ./QT2120/Arg_mvp_Qtouch.d ./QT2120/Arg_mvp_Qtouch.o ./QT2120/Arg_mvp_Qtouch.su ./QT2120/TouchWheel.d ./QT2120/TouchWheel.o ./QT2120/TouchWheel.su

.PHONY: clean-QT2120

