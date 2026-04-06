################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../LoRaMESH/src/adapters/linux/LinuxSerialTransport.cpp 

OBJS += \
./LoRaMESH/src/adapters/linux/LinuxSerialTransport.o 

CPP_DEPS += \
./LoRaMESH/src/adapters/linux/LinuxSerialTransport.d 


# Each subdirectory must supply rules for building sources it contributes
LoRaMESH/src/adapters/linux/%.o LoRaMESH/src/adapters/linux/%.su LoRaMESH/src/adapters/linux/%.cyclo: ../LoRaMESH/src/adapters/linux/%.cpp LoRaMESH/src/adapters/linux/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L412xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-LoRaMESH-2f-src-2f-adapters-2f-linux

clean-LoRaMESH-2f-src-2f-adapters-2f-linux:
	-$(RM) ./LoRaMESH/src/adapters/linux/LinuxSerialTransport.cyclo ./LoRaMESH/src/adapters/linux/LinuxSerialTransport.d ./LoRaMESH/src/adapters/linux/LinuxSerialTransport.o ./LoRaMESH/src/adapters/linux/LinuxSerialTransport.su

.PHONY: clean-LoRaMESH-2f-src-2f-adapters-2f-linux

