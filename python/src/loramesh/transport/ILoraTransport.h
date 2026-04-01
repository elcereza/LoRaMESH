#ifndef ILORATRANSPORT_H
#define ILORATRANSPORT_H

#include <stdint.h>
#include <stddef.h>

class ILoraTransport
{
public:
    virtual ~ILoraTransport() {}

    // Envia bytes (não-bloqueante ou bloqueante, depende do adapter).
    // Retorna quantos bytes foram aceitos/enviados.
    virtual size_t write(const uint8_t* data, size_t len) = 0;

    // Retorna quantos bytes estão disponíveis para leitura imediata.
    virtual size_t available() = 0;

    // Lê até maxLen bytes; deve retornar quantos bytes foram lidos.
    // Deve ser "rápido" (sem ficar travado esperando).
    virtual size_t read(uint8_t* buffer, size_t maxLen) = 0;

    // Tempo em ms (tipo millis). Pode vir de HAL_GetTick(), esp_timer, Arduino millis, etc.
    virtual uint32_t millis() = 0;
};

#endif