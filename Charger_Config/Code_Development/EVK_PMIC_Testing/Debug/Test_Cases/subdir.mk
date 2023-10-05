################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Test_Cases/pmic_chg_test_cases.c 

OBJS += \
./Test_Cases/pmic_chg_test_cases.o 

C_DEPS += \
./Test_Cases/pmic_chg_test_cases.d 


# Each subdirectory must supply rules for building sources it contributes
Test_Cases/%.o Test_Cases/%.su: ../Test_Cases/%.c Test_Cases/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F769xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I"C:/Projects/EVK_PMIC_Testing/Test_Cases" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Test_Cases

clean-Test_Cases:
	-$(RM) ./Test_Cases/pmic_chg_test_cases.d ./Test_Cases/pmic_chg_test_cases.o ./Test_Cases/pmic_chg_test_cases.su

.PHONY: clean-Test_Cases

