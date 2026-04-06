################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../LoRaMESH/src/core/LoRaMESH.cpp 

OBJS += \
./LoRaMESH/src/core/LoRaMESH.o 

CPP_DEPS += \
./LoRaMESH/src/core/LoRaMESH.d 


# Each subdirectory must supply rules for building sources it contributes
LoRaMESH/src/core/%.o LoRaMESH/src/core/%.su LoRaMESH/src/core/%.cyclo: ../LoRaMESH/src/core/%.cpp LoRaMESH/src/core/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L412xx -c -I../Core/Inc -I"/media/gustavo/nas/ssd/Lab/GitHub/LoRaMESH/examples/stm32/blinks/stm32l412kb_loramesh/LoRaMESH/src" -I"/media/gustavo/nas/ssd/Lab/GitHub/LoRaMESH/examples/stm32/blinks/stm32l412kb_loramesh/LoRaMESH/src/core" -I"/media/gustavo/nas/ssd/Lab/GitHub/LoRaMESH/examples/stm32/blinks/stm32l412kb_loramesh/LoRaMESH/src/adapters/stm32_hal" -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-LoRaMESH-2f-src-2f-core

clean-LoRaMESH-2f-src-2f-core:
	-$(RM) ./LoRaMESH/src/core/LoRaMESH.cyclo ./LoRaMESH/src/core/LoRaMESH.d ./LoRaMESH/src/core/LoRaMESH.o ./LoRaMESH/src/core/LoRaMESH.su

.PHONY: clean-LoRaMESH-2f-src-2f-core

