#pragma once

#ifdef _WIN32

#include "transport/ILoraTransport.h"
#include <windows.h>

class WindowsSerialTransport : public ILoraTransport
{
private:
    HANDLE hSerial;

public:
    WindowsSerialTransport(const char* port, int baud);

    bool begin();

    void updateBuffer();
    size_t write(const uint8_t* data, size_t len) override;
    size_t read(uint8_t* data, size_t len) override;
    size_t available() override;
    uint32_t millis() override;
};

#endif