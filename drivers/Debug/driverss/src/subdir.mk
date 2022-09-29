################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../driverss/src/gpio_driver.c 

OBJS += \
./driverss/src/gpio_driver.o 

C_DEPS += \
./driverss/src/gpio_driver.d 


# Each subdirectory must supply rules for building sources it contributes
driverss/src/%.o: ../driverss/src/%.c driverss/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32L4 -DSTM32 -DNUCLEO_L476RG -DSTM32L476RGTx -c -I"E:/new driver/drivers/driverss/inc" -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=soft -mthumb -o "$@"

clean: clean-driverss-2f-src

clean-driverss-2f-src:
	-$(RM) ./driverss/src/gpio_driver.d ./driverss/src/gpio_driver.o

.PHONY: clean-driverss-2f-src

