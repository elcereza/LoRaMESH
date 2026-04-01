#pragma once
#if defined(__linux__)

#include "transport/ILoraTransport.h"
#include <stdint.h>

class LinuxSerialTransport : public ILoraTransport
{
private:
    int fd;

public:
    LinuxSerialTransport(const char* device, int baud);

    bool begin();

    size_t write(const uint8_t* data, size_t len) override;

    size_t read(uint8_t* data, size_t len) override;

    size_t available() override;

    uint32_t millis() override;
};
#endif