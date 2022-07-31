################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../addons/src/pca9685.c 

C_DEPS += \
./addons/src/pca9685.d 

OBJS += \
./addons/src/pca9685.o 


# Each subdirectory must supply rules for building sources it contributes
addons/src/%.o addons/src/%.su: ../addons/src/%.c addons/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Matej/Desktop/ARM-A/software/addons/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-addons-2f-src

clean-addons-2f-src:
	-$(RM) ./addons/src/pca9685.d ./addons/src/pca9685.o ./addons/src/pca9685.su

.PHONY: clean-addons-2f-src

