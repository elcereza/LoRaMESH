#pragma once
#if defined(ESP_PLATFORM)

#include "transport/ILoraTransport.h"
extern "C" {
#include "driver/uart.h"
#include "esp_timer.h"
}

class EspIdfUartTransport : public ILoraTransport
{
public:
    explicit EspIdfUartTransport(uart_port_t port) : uart(port) {}

    size_t write(const uint8_t* data, size_t len) override
    {
        const int w = uart_write_bytes(uart, (const char*)data, (size_t)len);
        return (w < 0) ? 0 : (size_t)w;
    }

    size_t available() override
    {
        size_t buffered = 0;
        uart_get_buffered_data_len(uart, &buffered);
        return buffered;
    }

    size_t read(uint8_t* buffer, size_t maxLen) override
    {
        // timeout 0 ticks => não bloqueia
        const int r = uart_read_bytes(uart, buffer, maxLen, 0);
        return (r < 0) ? 0 : (size_t)r;
    }

    uint32_t millis() override
    {
        const int64_t us = esp_timer_get_time();
        return (uint32_t)(us / 1000);
    }

private:
    uart_port_t uart;
};

#endif // ESP_PLATFORM