#ifndef ILORATRANSPORT_H
#define ILORATRANSPORT_H

#include <stdint.h>
#include <stddef.h>

class ILoraTransport
{
public:
    virtual ~ILoraTransport() {}
    virtual size_t write(const uint8_t* data, size_t len) = 0;
    virtual size_t available() = 0;
    virtual size_t read(uint8_t* buffer, size_t maxLen) = 0;
    virtual uint32_t millis() = 0;
};

#endif