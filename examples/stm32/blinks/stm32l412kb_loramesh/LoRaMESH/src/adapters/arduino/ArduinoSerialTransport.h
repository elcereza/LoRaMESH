#pragma once

#if defined(ARDUINO)
#include "transport/ILoraTransport.h"
#include <Arduino.h>
#include <Stream.h>

class ArduinoSerialTransport : public ILoraTransport
{
public:
    explicit ArduinoSerialTransport(Stream& s) : serial(s) {}

    size_t write(const uint8_t* data, size_t len) override
    {
        return serial.write(data, len);
    }

    size_t available() override
    {
        return (size_t)serial.available();
    }

    size_t read(uint8_t* buffer, size_t maxLen) override
    {
        size_t i = 0;
        while (serial.available() > 0 && i < maxLen)
        {
            const int v = serial.read();
            if (v < 0) break;
            buffer[i++] = (uint8_t)v;
        }
        return i;
    }

    uint32_t millis() override
    {
        return ::millis();
    }

private:
    Stream& serial;
};

#endif