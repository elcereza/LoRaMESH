#pragma once

#include "core/LoRaMESH.h"

#include "transport/ILoraTransport.h"

#if defined(ARDUINO)
  #include "adapters/arduino/ArduinoSerialTransport.h"
#endif
/*
#if defined(ESP_PLATFORM)
  #include "adapters/espidf/EspIdfUartTransport.h"
#endif

#if defined(STM32) || defined(STM32F0xx) || defined(STM32F1xx)
  #include "adapters/stm32_hal/Stm32HalUartTransport.h"
#endif
*/
#if defined(__linux__)
  #include "adapters/linux/LinuxSerialTransport.h"
#endif