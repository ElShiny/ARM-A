################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../addons/src/3DEngine.c \
../addons/src/LED.c \
../addons/src/SCI.c \
../addons/src/XPT2046_touch.c \
../addons/src/buf.c \
../addons/src/joystick.c \
../addons/src/kbd.c \
../addons/src/lcd.c \
../addons/src/lcd_backlight.c \
../addons/src/lcd_ili9341.c \
../addons/src/matrix_func.c \
../addons/src/mcp3464.c \
../addons/src/mpu6050.c \
../addons/src/pca9685.c \
../addons/src/periodic_services.c \
../addons/src/timing_utils.c 

C_DEPS += \
./addons/src/3DEngine.d \
./addons/src/LED.d \
./addons/src/SCI.d \
./addons/src/XPT2046_touch.d \
./addons/src/buf.d \
./addons/src/joystick.d \
./addons/src/kbd.d \
./addons/src/lcd.d \
./addons/src/lcd_backlight.d \
./addons/src/lcd_ili9341.d \
./addons/src/matrix_func.d \
./addons/src/mcp3464.d \
./addons/src/mpu6050.d \
./addons/src/pca9685.d \
./addons/src/periodic_services.d \
./addons/src/timing_utils.d 

OBJS += \
./addons/src/3DEngine.o \
./addons/src/LED.o \
./addons/src/SCI.o \
./addons/src/XPT2046_touch.o \
./addons/src/buf.o \
./addons/src/joystick.o \
./addons/src/kbd.o \
./addons/src/lcd.o \
./addons/src/lcd_backlight.o \
./addons/src/lcd_ili9341.o \
./addons/src/matrix_func.o \
./addons/src/mcp3464.o \
./addons/src/mpu6050.o \
./addons/src/pca9685.o \
./addons/src/periodic_services.o \
./addons/src/timing_utils.o 


# Each subdirectory must supply rules for building sources it contributes
addons/src/%.o addons/src/%.su addons/src/%.cyclo: ../addons/src/%.c addons/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -DUSE_FULL_LL_DRIVER -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Matej/Documents/GitHub/ARM-A/software/addons/inc" -I"C:/Users/Matej/Documents/GitHub/ARM-A/software/addons" -O0 -ffunction-sections -fdata-sections -Wall -fcommon -Wmissing-braces -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-addons-2f-src

clean-addons-2f-src:
	-$(RM) ./addons/src/3DEngine.cyclo ./addons/src/3DEngine.d ./addons/src/3DEngine.o ./addons/src/3DEngine.su ./addons/src/LED.cyclo ./addons/src/LED.d ./addons/src/LED.o ./addons/src/LED.su ./addons/src/SCI.cyclo ./addons/src/SCI.d ./addons/src/SCI.o ./addons/src/SCI.su ./addons/src/XPT2046_touch.cyclo ./addons/src/XPT2046_touch.d ./addons/src/XPT2046_touch.o ./addons/src/XPT2046_touch.su ./addons/src/buf.cyclo ./addons/src/buf.d ./addons/src/buf.o ./addons/src/buf.su ./addons/src/joystick.cyclo ./addons/src/joystick.d ./addons/src/joystick.o ./addons/src/joystick.su ./addons/src/kbd.cyclo ./addons/src/kbd.d ./addons/src/kbd.o ./addons/src/kbd.su ./addons/src/lcd.cyclo ./addons/src/lcd.d ./addons/src/lcd.o ./addons/src/lcd.su ./addons/src/lcd_backlight.cyclo ./addons/src/lcd_backlight.d ./addons/src/lcd_backlight.o ./addons/src/lcd_backlight.su ./addons/src/lcd_ili9341.cyclo ./addons/src/lcd_ili9341.d ./addons/src/lcd_ili9341.o ./addons/src/lcd_ili9341.su ./addons/src/matrix_func.cyclo ./addons/src/matrix_func.d ./addons/src/matrix_func.o ./addons/src/matrix_func.su ./addons/src/mcp3464.cyclo ./addons/src/mcp3464.d ./addons/src/mcp3464.o ./addons/src/mcp3464.su ./addons/src/mpu6050.cyclo ./addons/src/mpu6050.d ./addons/src/mpu6050.o ./addons/src/mpu6050.su ./addons/src/pca9685.cyclo ./addons/src/pca9685.d ./addons/src/pca9685.o ./addons/src/pca9685.su ./addons/src/periodic_services.cyclo ./addons/src/periodic_services.d ./addons/src/periodic_services.o ./addons/src/periodic_services.su ./addons/src/timing_utils.cyclo ./addons/src/timing_utils.d ./addons/src/timing_utils.o ./addons/src/timing_utils.su

.PHONY: clean-addons-2f-src

