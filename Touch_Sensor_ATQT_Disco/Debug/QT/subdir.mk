################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../QT/Appfruits_QTouch.cpp \
../QT/Arg_mvp_Qtouch.cpp \
../QT/TouchWheel.cpp 

OBJS += \
./QT/Appfruits_QTouch.o \
./QT/Arg_mvp_Qtouch.o \
./QT/TouchWheel.o 

CPP_DEPS += \
./QT/Appfruits_QTouch.d \
./QT/Arg_mvp_Qtouch.d \
./QT/TouchWheel.d 


# Each subdirectory must supply rules for building sources it contributes
QT/%.o QT/%.su: ../QT/%.cpp QT/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F769xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I"D:/Mithlesh/NewProject/ATQT_Disco/QT" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-QT

clean-QT:
	-$(RM) ./QT/Appfruits_QTouch.d ./QT/Appfruits_QTouch.o ./QT/Appfruits_QTouch.su ./QT/Arg_mvp_Qtouch.d ./QT/Arg_mvp_Qtouch.o ./QT/Arg_mvp_Qtouch.su ./QT/TouchWheel.d ./QT/TouchWheel.o ./QT/TouchWheel.su

.PHONY: clean-QT

