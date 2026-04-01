#ifndef LORAMESH_H
#define LORAMESH_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <math.h>

#include "transport/ILoraTransport.h"

#define MAX_PAYLOAD_SIZE     232
#define MAX_BUFFER_SIZE      237

#define BW125                0x00
#define BW250                0x01
#define BW500                0x02

#define SF7                  0x07
#define SF8                  0x08
#define SF9                  0x09
#define SF10                 0x0A
#define SF11                 0x0B
#define SF12                 0x0C
#define SF_FSK               0x00

#define CR4_5                0x01
#define CR4_6                0x02
#define CR4_7                0x03
#define CR4_8                0x04

#define CLASS_A              0x00
#define CLASS_C              0x02

#define WINDOW_5s            0x00
#define WINDOW_10s           0x01
#define WINDOW_15s           0x02

#define LNOT_PULL            0x00
#define LPULLUP              0x01
#define LPULLDOWN            0x02

#define INOUT_DIGITAL_INPUT  0x00
#define INOUT_DIGITAL_OUTPUT 0x01
#define INOUT_ANALOG_INPUT   0x03

#ifndef INPUT
#define INPUT                0
#endif

#ifndef OUTPUT
#define OUTPUT               1
#endif

#ifndef INPUT_PULLUP
#define INPUT_PULLUP         7
#endif

#ifndef INPUT_PULLDOWN
#define INPUT_PULLDOWN       8
#endif

class LoRaMESH
{
public:
    typedef void (*DebugFn)(const char* msg);

    struct Frame
    {
        uint8_t buffer[MAX_BUFFER_SIZE];
        size_t  size;
        bool    command; // true = cmd transport, false = transparent transport
    };

public:
    // Mantive público como seu original
    uint8_t  bufferPayload[MAX_PAYLOAD_SIZE];
    uint8_t  payloadSize;

    uint16_t localId;
    uint32_t localUniqueId;
    uint8_t  command;
    bool     isMaster;

    Frame    frame;

    uint16_t deviceId;
    uint16_t deviceNet;
    uint32_t deviceUniqueId;

    uint32_t registered_password;

    uint8_t BW, SF, CR, LoRa_class, LoRa_window;

public:
    // transports
    LoRaMESH(ILoraTransport* cmdTransport, ILoraTransport* transpTransport = nullptr);

    // debug
    void setDebug(DebugFn fn, bool enable_hex_dump = false);

    // API
    void begin(bool debug = false);

    uint16_t ComputeCRC(const uint8_t* data_in, uint16_t length);

    bool prepareFrameCommand(uint16_t id, uint8_t cmd, const uint8_t* payload, uint8_t payloadSize);
    bool PrepareFrameTransp(uint16_t id, const uint8_t* payload, uint8_t payloadSize);

    bool sendPacket();

    bool receivePacketCommand(uint16_t* id, uint8_t* cmd, uint8_t* payload, uint8_t* payloadSize,
                              uint32_t timeout_ms, uint32_t inter_byte_timeout_ms = 30);

    bool receivePacketTransp(uint16_t* id,
                         uint8_t* payload,
                         uint8_t* payloadSize,
                         uint32_t timeout,
                         uint32_t interByteTimeout);

    bool localRead();

    bool setNetworkID(uint16_t id);
    bool setPassword(uint32_t password);

    bool getBPS(bool ignore_cmd = false);
    bool getClass(bool ignore_cmd = false);

    bool setBPS(uint8_t bandwidth = BW500, uint8_t spreading_factor = SF7, uint8_t coding_rate = CR4_5);
    bool setClass(uint8_t lora_class = CLASS_C, uint8_t lora_window = WINDOW_5s);

    bool pinMode(uint16_t id, uint8_t gpio, uint8_t inout, uint8_t logical_level = 0);
    void getGPIOStatus(int16_t id, uint8_t gpio);

    uint16_t analogRead(int16_t id, uint8_t gpio);
    uint8_t digitalRead(int16_t id, uint8_t gpio);
    bool digitalWrite(int16_t id, uint8_t gpio, uint8_t logical_level);

    int getNoise(uint16_t id, uint8_t select = 1);

    int getR1(uint16_t rawADC, int R2);
    double getTemp(uint16_t rawADC, int beta, int Rt = 10000, int R2 = 10000);

private:
    ILoraTransport* tCmd;
    ILoraTransport* tTransp;

    bool analogEnabled;

    DebugFn dbg;
    bool dbgHex;

private:
    void dbgPrint(const char* s);
    void dbgHexDump(const char* prefix, const uint8_t* data, size_t len);

    static void u16_to_le(uint16_t v, uint8_t out[2]);
    static uint16_t le_to_u16(const uint8_t in[2]);

    bool recvRaw(ILoraTransport* t, uint8_t* out, size_t* outLen, size_t maxLen,
                 uint32_t timeout_ms, uint32_t inter_byte_timeout_ms);
};

#endif