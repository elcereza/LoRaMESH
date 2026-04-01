#include "LoRaMESH.h"

LoRaMESH::LoRaMESH(ILoraTransport* cmdTransport, ILoraTransport* transpTransport)
: tCmd(cmdTransport),
  tTransp(transpTransport),
  analogEnabled(false),
  dbg(nullptr),
  dbgHex(false)
{
    memset(bufferPayload, 0, sizeof(bufferPayload));
    payloadSize = 0;

    localId = 0;
    localUniqueId = 0;
    command = 0;
    isMaster = false;

    frame.size = 0;
    frame.command = true;
    memset(frame.buffer, 0, sizeof(frame.buffer));

    deviceId = 0xFFFF;
    deviceNet = 0xFFFF;
    deviceUniqueId = 0xFFFFFFFF;

    registered_password = 0;

    BW = SF = CR = 0;
    LoRa_class = 0;
    LoRa_window = 0;
}

void LoRaMESH::setDebug(DebugFn fn, bool enable_hex_dump)
{
    dbg = fn;
    dbgHex = enable_hex_dump;
}

void LoRaMESH::dbgPrint(const char* s)
{
    if (dbg) dbg(s);
}

void LoRaMESH::dbgHexDump(const char* prefix, const uint8_t* data, size_t len)
{
    if (!dbg || !dbgHex) return;
    char line[128];
    size_t idx = 0;

    while (prefix && *prefix && idx < sizeof(line) - 1) line[idx++] = *prefix++;

    for (size_t i = 0; i < len; i++)
    {
        if (idx > sizeof(line) - 4) // flush
        {
            line[idx] = '\0';
            dbg(line);
            idx = 0;
        }

        static const char* hexd = "0123456789ABCDEF";
        uint8_t b = data[i];
        if (idx < sizeof(line) - 3)
        {
            line[idx++] = hexd[(b >> 4) & 0x0F];
            line[idx++] = hexd[b & 0x0F];
            line[idx++] = ' ';
        }
    }

    line[idx] = '\0';
    dbg(line);
}

void LoRaMESH::u16_to_le(uint16_t v, uint8_t out[2])
{
    out[0] = (uint8_t)(v & 0xFF);
    out[1] = (uint8_t)((v >> 8) & 0xFF);
}

uint16_t LoRaMESH::le_to_u16(const uint8_t in[2])
{
    return (uint16_t)in[0] | ((uint16_t)in[1] << 8);
}

void LoRaMESH::begin(bool debug)
{
    if (debug) dbgHex = true;

    localRead();
}

uint16_t LoRaMESH::ComputeCRC(const uint8_t* data_in, uint16_t length)
{
    uint16_t crc_calc = 0xC181;

    for (uint16_t i = 0; i < length; i++)
    {
        crc_calc ^= ((uint16_t)data_in[i]) & 0x00FF;

        for (uint8_t j = 0; j < 8; j++)
        {
            uint8_t bitbang = (uint8_t)(crc_calc & 0x01);
            crc_calc >>= 1;
            if (bitbang) crc_calc ^= 0xA001;
        }
    }
    return crc_calc & 0xFFFF;
}

bool LoRaMESH::prepareFrameCommand(uint16_t id, uint8_t cmd, const uint8_t* payload, uint8_t pSize)
{
    if (!tCmd) return false;
    if (pSize > MAX_PAYLOAD_SIZE) return false;

    const size_t total = (size_t)pSize + 5;
    if (total > MAX_BUFFER_SIZE) return false;

    frame.size = total;
    frame.command = true;

    frame.buffer[0] = (uint8_t)(id & 0xFF);
    frame.buffer[1] = (uint8_t)((id >> 8) & 0xFF);
    frame.buffer[2] = cmd;

    if (pSize > 0 && payload)
        memcpy(&frame.buffer[3], payload, pSize);

    const uint16_t crc = ComputeCRC(&frame.buffer[0], (uint16_t)(pSize + 3));
    frame.buffer[pSize + 3] = (uint8_t)(crc & 0xFF);
    frame.buffer[pSize + 4] = (uint8_t)((crc >> 8) & 0xFF);

    dbgHexDump("TX CMD: ", frame.buffer, frame.size);

    return sendPacket();
}

bool LoRaMESH::PrepareFrameTransp(uint16_t id, const uint8_t* payload, uint8_t pSize)
{
    if (pSize > MAX_PAYLOAD_SIZE) return false;

    const size_t total = (size_t)pSize + 2;
    if (total > MAX_BUFFER_SIZE) return false;

    frame.size = total;
    frame.command = false;

    frame.buffer[0] = (uint8_t)(id & 0xFF);
    frame.buffer[1] = (uint8_t)((id >> 8) & 0xFF); // FIX: não mask 0x03

    if (pSize > 0 && payload)
        memcpy(&frame.buffer[2], payload, pSize);

    dbgHexDump("TX TRP: ", frame.buffer, frame.size);

    return true;
}

bool LoRaMESH::sendPacket()
{
    if (frame.size == 0) return false;

    if (frame.command)
    {
        if (!tCmd) return false;
        return (tCmd->write(frame.buffer, frame.size) == frame.size);
    }
    else
    {
        if (!tTransp) return false;
        return (tTransp->write(frame.buffer, frame.size) == frame.size);
    }
}

bool LoRaMESH::recvRaw(ILoraTransport* t, uint8_t* out, size_t* outLen, size_t maxLen,
                      uint32_t timeout_ms, uint32_t inter_byte_timeout_ms)
{
    if (!t || !out || !outLen) return false;
    *outLen = 0;

    const uint32_t start = t->millis();
    uint32_t lastByteAt = 0;
    bool started = false;

    while (true)
    {
        const uint32_t now = t->millis();

        if (!started && (now - start) >= timeout_ms)
            break;

        if (started && (now - lastByteAt) >= inter_byte_timeout_ms)
            break;

        size_t avail = t->available();
        if (avail > 0)
        {
            if (*outLen >= maxLen) return false;

            uint8_t b = 0;
            size_t r = t->read(&b, 1);
            if (r == 1)
            {
                out[(*outLen)++] = b;
                started = true;
                lastByteAt = now;
            }
        }
    }

    return (*outLen > 0);
}

bool LoRaMESH::receivePacketCommand(uint16_t* id, uint8_t* cmd, uint8_t* payload, uint8_t* pSize,
                                   uint32_t timeout_ms, uint32_t inter_byte_timeout_ms)
{
    if (!tCmd || !id || !cmd || !payload || !pSize) return false;

    size_t n = 0;
    if (!recvRaw(tCmd, frame.buffer, &n, MAX_BUFFER_SIZE, timeout_ms, inter_byte_timeout_ms))
        return false;

    dbgHexDump("RX CMD: ", frame.buffer, n);

    if (n < 5) return false;

    const uint16_t rxCrc = (uint16_t)frame.buffer[n - 2] | ((uint16_t)frame.buffer[n - 1] << 8);
    const uint16_t calc = ComputeCRC(&frame.buffer[0], (uint16_t)(n - 2));
    if (calc != rxCrc) return false;

    *id = (uint16_t)frame.buffer[0] | ((uint16_t)frame.buffer[1] << 8);
    *cmd = frame.buffer[2];

    size_t payLen = n - 5;
    if (payLen > MAX_PAYLOAD_SIZE) return false;

    *pSize = (uint8_t)payLen;
    if (payLen > 0) memcpy(payload, &frame.buffer[3], payLen);

    return true;
}

bool LoRaMESH::receivePacketTransp(uint16_t* id, uint8_t* payload, uint8_t* pSize,
                                  uint32_t timeout_ms, uint32_t inter_byte_timeout_ms)
{
    if (!tTransp || !payload || !pSize) return false;

    size_t n = 0;
    if (!recvRaw(tTransp, frame.buffer, &n, MAX_BUFFER_SIZE, timeout_ms, inter_byte_timeout_ms))
        return false;

    dbgHexDump("RX TRP: ", frame.buffer, n);

    if (deviceId == 0)
    {
        if (!id) return false;
        if (n < 2) return false;

        *id = (uint16_t)frame.buffer[0] | ((uint16_t)frame.buffer[1] << 8);

        size_t payLen = n - 2;
        if (payLen > MAX_PAYLOAD_SIZE) return false;

        *pSize = (uint8_t)payLen;
        if (payLen > 0) memcpy(payload, &frame.buffer[2], payLen);
        return true;
    }
    else
    {
        if (n > MAX_PAYLOAD_SIZE) return false;
        *pSize = (uint8_t)n;
        if (n > 0) memcpy(payload, &frame.buffer[0], n);
        if (id) *id = 0;
        return true;
    }
}

bool LoRaMESH::localRead()
{
    uint8_t b = 0;
    bufferPayload[b] = 0x00;
    bufferPayload[++b] = 0x00;
    bufferPayload[++b] = 0x00;

    if (!prepareFrameCommand(0, 0xE2, bufferPayload, (uint8_t)(b + 1)))
        return false;

    if (!receivePacketCommand(&localId, &command, bufferPayload, &payloadSize, 1000, 50))
        return false;

    if (command != 0xE2) return false;
    if (payloadSize < 6) return false;

    registered_password = ((uint32_t)bufferPayload[1] << 8) | (uint32_t)bufferPayload[0];

    localUniqueId = bufferPayload[2];
    localUniqueId = bufferPayload[3] + (localUniqueId << 8);
    localUniqueId = bufferPayload[4] + (localUniqueId << 8);
    localUniqueId = bufferPayload[5] + (localUniqueId << 8);

    return true;
}

bool LoRaMESH::setNetworkID(uint16_t id)
{
    if(id > 2047)
        return false;
    uint8_t b = 0;

    bufferPayload[b] = 0x00;
    bufferPayload[++b] = 0x00;
    bufferPayload[++b] = (uint8_t)(localUniqueId >> 24);
    bufferPayload[++b] = (uint8_t)(localUniqueId >> 16);
    bufferPayload[++b] = (uint8_t)(localUniqueId >> 8);
    bufferPayload[++b] = (uint8_t)(localUniqueId & 0xFF);
    bufferPayload[++b] = 0x00;
    bufferPayload[++b] = 0x00;
    bufferPayload[++b] = 0x00;
    bufferPayload[++b] = 0x00;
    bufferPayload[++b] = 0x00;
    bufferPayload[++b] = 0x04;

    if (!prepareFrameCommand(id, 0xCA, bufferPayload, (uint8_t)(b + 1)))
        return false;

    if (!receivePacketCommand(&localId, &command, bufferPayload, &payloadSize, 1000, 50))
        return false;

    return (command == 0xCA);
}

bool LoRaMESH::setPassword(uint32_t password)
{
    uint8_t b = 0;

    bufferPayload[b] = 0x04;
    bufferPayload[++b] = (uint8_t)(password & 0xFF);
    bufferPayload[++b] = (uint8_t)((password >> 8) & 0xFF);
    bufferPayload[++b] = (uint8_t)((password >> 16) & 0xFF);
    bufferPayload[++b] = (uint8_t)((password >> 24) & 0xFF);

    if (!prepareFrameCommand(localId, 0xCD, bufferPayload, (uint8_t)(b + 1)))
        return false;

    uint32_t expected16 = ((uint32_t)bufferPayload[2] << 8) | (uint32_t)bufferPayload[1];

    if (!receivePacketCommand(&localId, &command, bufferPayload, &payloadSize, 1000, 50))
        return false;

    if (command != 0xCD) return false;
    if (!localRead()) return false;
    return (expected16 == registered_password);
}

bool LoRaMESH::getBPS(bool ignore_cmd)
{
    uint8_t b = 0;

    if (!ignore_cmd)
    {
        bufferPayload[b] = 0x00;
        bufferPayload[++b] = 0x01;
        bufferPayload[++b] = 0x00;

        if (!prepareFrameCommand(localId, 0xD6, bufferPayload, (uint8_t)(b + 1)))
            return false;
    }

    if (!receivePacketCommand(&localId, &command, bufferPayload, &payloadSize, 1000, 50))
        return false;

    if (command != 0xD6) return false;
    if (payloadSize < 5) return false;

    BW = bufferPayload[2];
    SF = bufferPayload[3];
    CR = bufferPayload[4];
    return true;
}

bool LoRaMESH::getClass(bool ignore_cmd)
{
    uint8_t b = 0;

    if (!ignore_cmd)
    {
        bufferPayload[b] = 0x00;
        bufferPayload[++b] = 0xFF;
        bufferPayload[++b] = 0x00;
        bufferPayload[++b] = 0x00;

        if (!prepareFrameCommand(localId, 0xC1, bufferPayload, (uint8_t)(b + 1)))
            return false;
    }

    if (!receivePacketCommand(&localId, &command, bufferPayload, &payloadSize, 1000, 50))
        return false;

    if (command != 0xC1) return false;
    if (payloadSize < 4) return false;

    LoRa_class = bufferPayload[2];
    LoRa_window = bufferPayload[3];
    return true;
}

bool LoRaMESH::setBPS(uint8_t bandwidth, uint8_t spreading_factor, uint8_t coding_rate)
{
    if (bandwidth > BW500) return false;

    if (!(spreading_factor == SF_FSK || (spreading_factor >= SF7 && spreading_factor <= SF12)))
        return false;

    if (coding_rate < CR4_5 || coding_rate > CR4_8)
        return false;

    uint8_t b = 0;
    bufferPayload[b] = 0x01;
    bufferPayload[++b] = 0x14;
    bufferPayload[++b] = bandwidth;
    bufferPayload[++b] = spreading_factor;
    bufferPayload[++b] = coding_rate;

    if (!prepareFrameCommand(localId, 0xD6, bufferPayload, (uint8_t)(b + 1)))
        return false;

    if (!getBPS(true)) return false;

    return (BW == bandwidth && SF == spreading_factor && CR == coding_rate);
}

bool LoRaMESH::setClass(uint8_t lora_class, uint8_t lora_window)
{
    if (!(lora_class == CLASS_A || lora_class == CLASS_C)) return false;
    if (lora_window > WINDOW_15s) return false;

    uint8_t b = 0;
    bufferPayload[b] = 0x00;
    bufferPayload[++b] = lora_class;
    bufferPayload[++b] = lora_window;
    bufferPayload[++b] = 0x00;

    if (!prepareFrameCommand(localId, 0xC1, bufferPayload, (uint8_t)(b + 1)))
        return false;

    if (!getClass(true)) return false;

    return (LoRa_class == lora_class && LoRa_window == lora_window);
}

bool LoRaMESH::pinMode(uint16_t id, uint8_t gpio, uint8_t inout, uint8_t logical_level)
{
    if (gpio > 0x07) return false;

    uint8_t pull = LNOT_PULL;

    switch (inout)
    {
        case INPUT:
            inout = INOUT_DIGITAL_INPUT;
            pull = LNOT_PULL;
            break;
        case INPUT_PULLUP:
            inout = INOUT_DIGITAL_INPUT;
            pull = LPULLUP;
            break;
        case INPUT_PULLDOWN:
            inout = INOUT_DIGITAL_INPUT;
            pull = LPULLDOWN;
            break;
        case OUTPUT:
            inout = INOUT_DIGITAL_OUTPUT;
            pull = LNOT_PULL;
            break;
        default:
            if (!(inout == INOUT_DIGITAL_INPUT || inout == INOUT_DIGITAL_OUTPUT || inout == INOUT_ANALOG_INPUT))
                return false;
            break;
    }

    uint8_t b = 0;

    if (gpio > 4 && gpio < 7 && inout == INOUT_DIGITAL_INPUT)
    {
        bufferPayload[b] = 0x02;
        bufferPayload[++b] = 0x00;
        bufferPayload[++b] = gpio;
        bufferPayload[++b] = 0x00;
        bufferPayload[++b] = INOUT_ANALOG_INPUT;
        analogEnabled = true;
    }
    else
    {
        bufferPayload[b] = 0x02;
        bufferPayload[++b] = gpio;
        bufferPayload[++b] = pull;
        bufferPayload[++b] = inout;
        bufferPayload[++b] = logical_level;
    }

    if (!prepareFrameCommand(id, 0xC2, bufferPayload, (uint8_t)(b + 1)))
        return false;

    if (!receivePacketCommand(&localId, &command, bufferPayload, &payloadSize, 1000, 50))
        return false;

    return (command == 0xC2);
}

void LoRaMESH::getGPIOStatus(int16_t id, uint8_t gpio)
{
    uint8_t b = 0;

    bufferPayload[b] = 0x00;
    bufferPayload[++b] = gpio;
    bufferPayload[++b] = 0x00;
    bufferPayload[++b] = 0x00;

    (void)prepareFrameCommand((uint16_t)id, 0xC2, bufferPayload, (uint8_t)(b + 1));
}

uint16_t LoRaMESH::analogRead(int16_t id, uint8_t gpio)
{
    for (uint8_t i = 0; i < 3; ++i)
    {
        getGPIOStatus(id, gpio);

        if (receivePacketCommand(&localId, &command, bufferPayload, &payloadSize, 1000, 50))
        {
            if (command == 0xC2 && payloadSize >= 5)
            {
                uint16_t rawAnalog = ((uint16_t)bufferPayload[3] << 8) | (uint16_t)bufferPayload[4];
                return rawAnalog;
            }
        }
    }
    return 0;
}

uint8_t LoRaMESH::digitalRead(int16_t id, uint8_t gpio)
{
    getGPIOStatus(id, gpio);

    if (analogEnabled && gpio > 4 && gpio < 7)
    {
        uint16_t v = analogRead(id, gpio);
        return (v >= (4096 / 2)) ? 1 : 0;
    }

    if (receivePacketCommand(&localId, &command, bufferPayload, &payloadSize, 1000, 50))
    {
        if (command == 0xC2 && payloadSize >= 5)
            return bufferPayload[4];
    }

    return 0;
}

bool LoRaMESH::digitalWrite(int16_t id, uint8_t gpio, uint8_t logical_level)
{
    if (gpio > 0x07) return false;
    if (analogEnabled && gpio > 4 && gpio < 7) return false;

    for (uint8_t i = 0; i < 3; ++i)
    {
        uint8_t b = 0;

        bufferPayload[b] = 0x01;
        bufferPayload[++b] = gpio;
        bufferPayload[++b] = logical_level;
        bufferPayload[++b] = 0x00;

        if (!prepareFrameCommand((uint16_t)id, 0xC2, bufferPayload, (uint8_t)(b + 1)))
            continue;

        if (receivePacketCommand(&localId, &command, bufferPayload, &payloadSize, 1000, 50))
        {
            if (command == 0xC2 && payloadSize >= 4)
            {
                if (bufferPayload[2] == gpio && bufferPayload[3] == logical_level)
                    return true;
            }
        }
    }

    return false;
}

int LoRaMESH::getNoise(uint16_t id, uint8_t select)
{
    uint8_t b = 0;
    bufferPayload[b] = 0;
    bufferPayload[++b] = 0;
    bufferPayload[++b] = 0;

    if (select > 2) select = 2;

    if (!prepareFrameCommand(id, 0xD8, bufferPayload, (uint8_t)(b + 1)))
        return 255;

    uint16_t rid = 0;
    uint8_t rcmd = 0;
    uint8_t psz = 0;

    if (!receivePacketCommand(&rid, &rcmd, bufferPayload, &psz, 270, 50))
        return 255;

    if (rcmd != 0xD8) return 255;
    if (psz <= select) return 255;

    return bufferPayload[select];
}

int LoRaMESH::getR1(uint16_t rawADC, int R2)
{
    if (rawADC == 0) return 0x7FFFFFFF;
    if (rawADC >= 4096) return 0;
    return (int)((int64_t)R2 * (4096 - rawADC) / rawADC);
}

double LoRaMESH::getTemp(uint16_t rawADC, int beta, int Rt, int R2)
{
    double R1 = (double)getR1(rawADC, R2);
    if (R1 <= 0.0) return -273.15;

    double rx = (double)Rt * exp(-(double)beta / (273.0 + 25.0));
    double t  = (double)beta / log(R1 / rx);
    return t - 273.0;
}