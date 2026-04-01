#ifdef _WIN32

#include "adapters/windows/WindowsSerialTransport.h"
#include <chrono>
#include <vector>

std::vector<uint8_t> rxBuffer;

WindowsSerialTransport::WindowsSerialTransport(const char* port, int baud)
{
    hSerial = CreateFileA(
        port,
        GENERIC_READ | GENERIC_WRITE,
        0,
        0,
        OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL,
        0
    );

    if (hSerial == INVALID_HANDLE_VALUE)
        return;

    SetupComm(hSerial, 4096, 4096);

    DCB dcb = {0};
    dcb.DCBlength = sizeof(dcb);

    if (!GetCommState(hSerial, &dcb))
    {
        CloseHandle(hSerial);
        hSerial = INVALID_HANDLE_VALUE;
        return;
    }

    dcb.BaudRate = baud;
    dcb.ByteSize = 8;
    dcb.StopBits = ONESTOPBIT;
    dcb.Parity   = NOPARITY;
    dcb.fDtrControl = DTR_CONTROL_ENABLE;
    dcb.fRtsControl = RTS_CONTROL_ENABLE;

    SetCommState(hSerial, &dcb);

    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout         = 50;
    timeouts.ReadTotalTimeoutConstant    = 50;
    timeouts.ReadTotalTimeoutMultiplier  = 10;

    SetCommTimeouts(hSerial, &timeouts);

    PurgeComm(hSerial, PURGE_RXCLEAR | PURGE_TXCLEAR);
}

bool WindowsSerialTransport::begin()
{
    return hSerial != INVALID_HANDLE_VALUE;
}

void WindowsSerialTransport::updateBuffer()
{
    if (hSerial == INVALID_HANDLE_VALUE) return;

    DWORD errors;
    COMSTAT status;

    if (!ClearCommError(hSerial, &errors, &status))
        return;

    if (status.cbInQue == 0)
        return;

    std::vector<uint8_t> temp(status.cbInQue);

    DWORD bytesRead = 0;
    if (ReadFile(hSerial, temp.data(), status.cbInQue, &bytesRead, NULL))
    {
        rxBuffer.insert(rxBuffer.end(), temp.begin(), temp.begin() + bytesRead);
    }
}

size_t WindowsSerialTransport::write(const uint8_t* data, size_t len)
{
    if (hSerial == INVALID_HANDLE_VALUE) return 0;

    DWORD bytesWritten;
    if (!WriteFile(hSerial, data, (DWORD)len, &bytesWritten, NULL))
    {
        return 0;
    }

    return (size_t)bytesWritten;
}

size_t WindowsSerialTransport::read(uint8_t* data, size_t len)
{
    size_t timeout = millis() + 100; // 100ms

    while (rxBuffer.size() == 0 && millis() < timeout)
    {
        updateBuffer();
        Sleep(1);
    }

    size_t toRead = (len < rxBuffer.size()) ? len : rxBuffer.size();

    if (toRead == 0) return 0;

    memcpy(data, rxBuffer.data(), toRead);
    rxBuffer.erase(rxBuffer.begin(), rxBuffer.begin() + toRead);

    return toRead;
}

size_t WindowsSerialTransport::available()
{
    if (hSerial == INVALID_HANDLE_VALUE) return 0;

    updateBuffer();

    // força pequena espera para dar tempo do dado chegar
    if (rxBuffer.empty())
    {
        Sleep(10);
        updateBuffer();
    }

    return rxBuffer.size();
}

uint32_t WindowsSerialTransport::millis()
{
    using namespace std::chrono;
    static auto start = steady_clock::now();
    auto now = steady_clock::now();

    return (uint32_t)duration_cast<milliseconds>(now - start).count();
}

#endif