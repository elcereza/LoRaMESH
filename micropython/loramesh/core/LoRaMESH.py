import math

from ..constants import (
    MAX_PAYLOAD_SIZE,
    MAX_BUFFER_SIZE,
    BW500,
    SF7,
    SF12,
    SF_FSK,
    CR4_5,
    CR4_8,
    CLASS_A,
    CLASS_C,
    WINDOW_5s,
    WINDOW_15s,
    LNOT_PULL,
    LPULLUP,
    LPULLDOWN,
    INOUT_DIGITAL_INPUT,
    INOUT_DIGITAL_OUTPUT,
    INOUT_ANALOG_INPUT,
    INPUT,
    OUTPUT,
    INPUT_PULLUP,
    INPUT_PULLDOWN,
)
from .Frame import Frame
from .utils import ticks_diff


class LoRaMESH:
    def __init__(self, cmdTransport, transpTransport=None):
        self.tCmd = cmdTransport
        self.tTransp = transpTransport if transpTransport is not None else cmdTransport

        self.analogEnabled = False

        self.dbg = None
        self.dbgHex = False

        self.bufferPayload = bytearray(MAX_PAYLOAD_SIZE)
        self.payloadSize = 0

        self.localId = 0
        self.localUniqueId = 0
        self.command = 0
        self.isMaster = False

        self.frame = Frame()

        self.deviceId = 0xFFFF
        self.deviceNet = 0xFFFF
        self.deviceUniqueId = 0xFFFFFFFF

        self.registered_password = 0

        self.BW = 0
        self.SF = 0
        self.CR = 0
        self.LoRa_class = 0
        self.LoRa_window = 0

    def setDebug(self, fn, enable_hex_dump=False):
        self.dbg = fn
        self.dbgHex = enable_hex_dump

    def dbgPrint(self, s):
        if self.dbg:
            self.dbg(s)

    def dbgHexDump(self, prefix, data, length):
        if (not self.dbg) or (not self.dbgHex):
            return

        line = ""
        idx = 0

        if prefix:
            while idx < (128 - 1) and idx < len(prefix):
                line += prefix[idx]
                idx += 1

        hexd = "0123456789ABCDEF"

        for i in range(length):
            if idx > (128 - 4):
                self.dbg(line)
                line = ""
                idx = 0

            b = data[i]
            if idx < (128 - 3):
                line += hexd[(b >> 4) & 0x0F]
                line += hexd[b & 0x0F]
                line += " "
                idx += 3

        self.dbg(line)

    @staticmethod
    def u16_to_le(v, out):
        out[0] = v & 0xFF
        out[1] = (v >> 8) & 0xFF

    @staticmethod
    def le_to_u16(data_in):
        return data_in[0] | (data_in[1] << 8)

    def begin(self, debug=False):
        if debug:
            self.dbgHex = True

        self.localRead()

    def ComputeCRC(self, data_in, length):
        crc_calc = 0xC181

        for i in range(length):
            crc_calc ^= data_in[i] & 0x00FF

            for _ in range(8):
                bitbang = crc_calc & 0x01
                crc_calc >>= 1
                if bitbang:
                    crc_calc ^= 0xA001

        return crc_calc & 0xFFFF

    def prepareFrameCommand(self, dev_id, cmd, payload, payloadSize):
        if self.tCmd is None:
            return False

        if payloadSize > MAX_PAYLOAD_SIZE:
            return False

        if payloadSize > 0:
            if payload is None:
                return False
            if len(payload) < payloadSize:
                return False

        total = payloadSize + 5
        if total > MAX_BUFFER_SIZE:
            return False

        self.frame.size = total
        self.frame.command = True

        self.frame.buffer[0] = dev_id & 0xFF
        self.frame.buffer[1] = (dev_id >> 8) & 0xFF
        self.frame.buffer[2] = cmd & 0xFF

        if payloadSize > 0:
            for i in range(payloadSize):
                self.frame.buffer[3 + i] = payload[i]

        crc = self.ComputeCRC(self.frame.buffer, payloadSize + 3)
        self.frame.buffer[payloadSize + 3] = crc & 0xFF
        self.frame.buffer[payloadSize + 4] = (crc >> 8) & 0xFF

        self.dbgHexDump("TX CMD: ", self.frame.buffer, self.frame.size)

        return self.sendPacket()

    def PrepareFrameTransp(self, dev_id, payload, payloadSize):
        if payloadSize > MAX_PAYLOAD_SIZE:
            return False

        if payloadSize > 0:
            if payload is None:
                return False
            if len(payload) < payloadSize:
                return False

        total = payloadSize + 2
        if total > MAX_BUFFER_SIZE:
            return False

        self.frame.size = total
        self.frame.command = False

        self.frame.buffer[0] = dev_id & 0xFF
        self.frame.buffer[1] = (dev_id >> 8) & 0xFF

        if payloadSize > 0:
            for i in range(payloadSize):
                self.frame.buffer[2 + i] = payload[i]

        self.dbgHexDump("TX TRP: ", self.frame.buffer, self.frame.size)

        return True

    def sendPacket(self):
        if self.frame.size == 0:
            return False

        if self.frame.command:
            if self.tCmd is None:
                return False
            return self.tCmd.write(self.frame.buffer, self.frame.size) == self.frame.size

        if self.tTransp is None:
            return False
        return self.tTransp.write(self.frame.buffer, self.frame.size) == self.frame.size

    def recvRaw(self, t, out, outLenRef, maxLen, timeout_ms, inter_byte_timeout_ms):
        if t is None or out is None or outLenRef is None:
            return False

        outLenRef[0] = 0

        start = t.millis()
        lastByteAt = 0
        started = False
        one = bytearray(1)

        while True:
            now = t.millis()

            if (not started) and (ticks_diff(now, start) >= timeout_ms):
                break

            if started and (ticks_diff(now, lastByteAt) >= inter_byte_timeout_ms):
                break

            avail = t.available()
            if avail > 0:
                if outLenRef[0] >= maxLen:
                    return False

                r = t.read(one, 1)
                if r == 1:
                    out[outLenRef[0]] = one[0]
                    outLenRef[0] += 1
                    started = True
                    lastByteAt = now

        return outLenRef[0] > 0

    def receivePacketCommand(self, idRef, cmdRef, payload, pSizeRef, timeout_ms, inter_byte_timeout_ms=30):
        if self.tCmd is None or idRef is None or cmdRef is None or payload is None or pSizeRef is None:
            return False

        nRef = [0]
        if not self.recvRaw(self.tCmd, self.frame.buffer, nRef, MAX_BUFFER_SIZE, timeout_ms, inter_byte_timeout_ms):
            return False

        n = nRef[0]

        self.dbgHexDump("RX CMD: ", self.frame.buffer, n)

        if n < 5:
            return False

        rxCrc = self.frame.buffer[n - 2] | (self.frame.buffer[n - 1] << 8)
        calc = self.ComputeCRC(self.frame.buffer, n - 2)
        if calc != rxCrc:
            return False

        idRef[0] = self.frame.buffer[0] | (self.frame.buffer[1] << 8)
        cmdRef[0] = self.frame.buffer[2]

        payLen = n - 5
        if payLen > MAX_PAYLOAD_SIZE:
            return False

        pSizeRef[0] = payLen
        if payLen > 0:
            for i in range(payLen):
                payload[i] = self.frame.buffer[3 + i]

        return True

    def receivePacketTransp(self, idRef, payload, pSizeRef, timeout_ms, inter_byte_timeout_ms):
        if self.tTransp is None or payload is None or pSizeRef is None:
            return False

        nRef = [0]
        if not self.recvRaw(self.tTransp, self.frame.buffer, nRef, MAX_BUFFER_SIZE, timeout_ms, inter_byte_timeout_ms):
            return False

        n = nRef[0]

        self.dbgHexDump("RX TRP: ", self.frame.buffer, n)

        if self.deviceId == 0:
            if idRef is None:
                return False
            if n < 2:
                return False

            idRef[0] = self.frame.buffer[0] | (self.frame.buffer[1] << 8)

            payLen = n - 2
            if payLen > MAX_PAYLOAD_SIZE:
                return False

            pSizeRef[0] = payLen
            if payLen > 0:
                for i in range(payLen):
                    payload[i] = self.frame.buffer[2 + i]
            return True

        if n > MAX_PAYLOAD_SIZE:
            return False

        pSizeRef[0] = n
        if n > 0:
            for i in range(n):
                payload[i] = self.frame.buffer[i]
        if idRef is not None:
            idRef[0] = 0
        return True

    def localRead(self):
        b = 0
        self.bufferPayload[b] = 0x00
        b += 1
        self.bufferPayload[b] = 0x00
        b += 1
        self.bufferPayload[b] = 0x00

        if not self.prepareFrameCommand(0, 0xE2, self.bufferPayload, b + 1):
            return False

        idRef = [0]
        cmdRef = [0]
        pSizeRef = [0]

        if not self.receivePacketCommand(idRef, cmdRef, self.bufferPayload, pSizeRef, 1000, 50):
            return False

        self.localId = idRef[0]
        self.command = cmdRef[0]
        self.payloadSize = pSizeRef[0]

        if self.command != 0xE2:
            return False
        if self.payloadSize < 6:
            return False

        self.registered_password = (self.bufferPayload[1] << 8) | self.bufferPayload[0]

        self.localUniqueId = self.bufferPayload[2]
        self.localUniqueId = self.bufferPayload[3] + (self.localUniqueId << 8)
        self.localUniqueId = self.bufferPayload[4] + (self.localUniqueId << 8)
        self.localUniqueId = self.bufferPayload[5] + (self.localUniqueId << 8)

        return True

    def setNetworkID(self, dev_id):
        if dev_id > 2047:
            return False

        b = 0
        self.bufferPayload[b] = 0x00
        b += 1
        self.bufferPayload[b] = 0x00
        b += 1
        self.bufferPayload[b] = (self.localUniqueId >> 24) & 0xFF
        b += 1
        self.bufferPayload[b] = (self.localUniqueId >> 16) & 0xFF
        b += 1
        self.bufferPayload[b] = (self.localUniqueId >> 8) & 0xFF
        b += 1
        self.bufferPayload[b] = self.localUniqueId & 0xFF
        b += 1
        self.bufferPayload[b] = 0x00
        b += 1
        self.bufferPayload[b] = 0x00
        b += 1
        self.bufferPayload[b] = 0x00
        b += 1
        self.bufferPayload[b] = 0x00
        b += 1
        self.bufferPayload[b] = 0x00
        b += 1
        self.bufferPayload[b] = 0x04

        if not self.prepareFrameCommand(dev_id, 0xCA, self.bufferPayload, b + 1):
            return False

        idRef = [0]
        cmdRef = [0]
        pSizeRef = [0]

        if not self.receivePacketCommand(idRef, cmdRef, self.bufferPayload, pSizeRef, 1000, 50):
            return False

        self.localId = idRef[0]
        self.command = cmdRef[0]
        self.payloadSize = pSizeRef[0]

        return self.command == 0xCA

    def setPassword(self, password):
        b = 0

        self.bufferPayload[b] = 0x04
        b += 1
        self.bufferPayload[b] = password & 0xFF
        b += 1
        self.bufferPayload[b] = (password >> 8) & 0xFF
        b += 1
        self.bufferPayload[b] = (password >> 16) & 0xFF
        b += 1
        self.bufferPayload[b] = (password >> 24) & 0xFF

        if not self.prepareFrameCommand(self.localId, 0xCD, self.bufferPayload, b + 1):
            return False

        expected16 = (self.bufferPayload[2] << 8) | self.bufferPayload[1]

        idRef = [0]
        cmdRef = [0]
        pSizeRef = [0]

        if not self.receivePacketCommand(idRef, cmdRef, self.bufferPayload, pSizeRef, 1000, 50):
            return False

        self.localId = idRef[0]
        self.command = cmdRef[0]
        self.payloadSize = pSizeRef[0]

        if self.command != 0xCD:
            return False
        if not self.localRead():
            return False

        return expected16 == self.registered_password

    def getBPS(self, ignore_cmd=False):
        b = 0

        if not ignore_cmd:
            self.bufferPayload[b] = 0x00
            b += 1
            self.bufferPayload[b] = 0x01
            b += 1
            self.bufferPayload[b] = 0x00

            if not self.prepareFrameCommand(self.localId, 0xD6, self.bufferPayload, b + 1):
                return False

        idRef = [0]
        cmdRef = [0]
        pSizeRef = [0]

        if not self.receivePacketCommand(idRef, cmdRef, self.bufferPayload, pSizeRef, 1000, 50):
            return False

        self.localId = idRef[0]
        self.command = cmdRef[0]
        self.payloadSize = pSizeRef[0]

        if self.command != 0xD6:
            return False
        if self.payloadSize < 5:
            return False

        self.BW = self.bufferPayload[2]
        self.SF = self.bufferPayload[3]
        self.CR = self.bufferPayload[4]
        return True

    def getClass(self, ignore_cmd=False):
        b = 0

        if not ignore_cmd:
            self.bufferPayload[b] = 0x00
            b += 1
            self.bufferPayload[b] = 0xFF
            b += 1
            self.bufferPayload[b] = 0x00
            b += 1
            self.bufferPayload[b] = 0x00

            if not self.prepareFrameCommand(self.localId, 0xC1, self.bufferPayload, b + 1):
                return False

        idRef = [0]
        cmdRef = [0]
        pSizeRef = [0]

        if not self.receivePacketCommand(idRef, cmdRef, self.bufferPayload, pSizeRef, 1000, 50):
            return False

        self.localId = idRef[0]
        self.command = cmdRef[0]
        self.payloadSize = pSizeRef[0]

        if self.command != 0xC1:
            return False
        if self.payloadSize < 4:
            return False

        self.LoRa_class = self.bufferPayload[2]
        self.LoRa_window = self.bufferPayload[3]
        return True

    def setBPS(self, bandwidth=BW500, spreading_factor=SF7, coding_rate=CR4_5):
        if bandwidth > BW500:
            return False

        if not (spreading_factor == SF_FSK or (spreading_factor >= SF7 and spreading_factor <= SF12)):
            return False

        if coding_rate < CR4_5 or coding_rate > CR4_8:
            return False

        b = 0
        self.bufferPayload[b] = 0x01
        b += 1
        self.bufferPayload[b] = 0x14
        b += 1
        self.bufferPayload[b] = bandwidth
        b += 1
        self.bufferPayload[b] = spreading_factor
        b += 1
        self.bufferPayload[b] = coding_rate

        if not self.prepareFrameCommand(self.localId, 0xD6, self.bufferPayload, b + 1):
            return False

        if not self.getBPS(True):
            return False

        return self.BW == bandwidth and self.SF == spreading_factor and self.CR == coding_rate

    def setClass(self, lora_class=CLASS_C, lora_window=WINDOW_5s):
        if not (lora_class == CLASS_A or lora_class == CLASS_C):
            return False
        if lora_window > WINDOW_15s:
            return False

        b = 0
        self.bufferPayload[b] = 0x00
        b += 1
        self.bufferPayload[b] = lora_class
        b += 1
        self.bufferPayload[b] = lora_window
        b += 1
        self.bufferPayload[b] = 0x00

        if not self.prepareFrameCommand(self.localId, 0xC1, self.bufferPayload, b + 1):
            return False

        if not self.getClass(True):
            return False

        return self.LoRa_class == lora_class and self.LoRa_window == lora_window

    def pinMode(self, dev_id, gpio, inout, logical_level=0):
        if gpio > 0x07:
            return False

        pull = LNOT_PULL

        if inout == INPUT:
            inout = INOUT_DIGITAL_INPUT
            pull = LNOT_PULL
        elif inout == INPUT_PULLUP:
            inout = INOUT_DIGITAL_INPUT
            pull = LPULLUP
        elif inout == INPUT_PULLDOWN:
            inout = INOUT_DIGITAL_INPUT
            pull = LPULLDOWN
        elif inout == OUTPUT:
            inout = INOUT_DIGITAL_OUTPUT
            pull = LNOT_PULL
        else:
            if not (
                inout == INOUT_DIGITAL_INPUT
                or inout == INOUT_DIGITAL_OUTPUT
                or inout == INOUT_ANALOG_INPUT
            ):
                return False

        b = 0

        if gpio > 4 and gpio < 7 and inout == INOUT_DIGITAL_INPUT:
            self.bufferPayload[b] = 0x02
            b += 1
            self.bufferPayload[b] = 0x00
            b += 1
            self.bufferPayload[b] = gpio
            b += 1
            self.bufferPayload[b] = 0x00
            b += 1
            self.bufferPayload[b] = INOUT_ANALOG_INPUT
            self.analogEnabled = True
        else:
            self.bufferPayload[b] = 0x02
            b += 1
            self.bufferPayload[b] = gpio
            b += 1
            self.bufferPayload[b] = pull
            b += 1
            self.bufferPayload[b] = inout
            b += 1
            self.bufferPayload[b] = logical_level

        if not self.prepareFrameCommand(dev_id, 0xC2, self.bufferPayload, b + 1):
            return False

        idRef = [0]
        cmdRef = [0]
        pSizeRef = [0]

        if not self.receivePacketCommand(idRef, cmdRef, self.bufferPayload, pSizeRef, 1000, 50):
            return False

        self.localId = idRef[0]
        self.command = cmdRef[0]
        self.payloadSize = pSizeRef[0]

        return self.command == 0xC2

    def getGPIOStatus(self, dev_id, gpio):
        b = 0

        self.bufferPayload[b] = 0x00
        b += 1
        self.bufferPayload[b] = gpio
        b += 1
        self.bufferPayload[b] = 0x00
        b += 1
        self.bufferPayload[b] = 0x00

        self.prepareFrameCommand(dev_id & 0xFFFF, 0xC2, self.bufferPayload, b + 1)

    def analogRead(self, dev_id, gpio):
        for _ in range(3):
            self.getGPIOStatus(dev_id, gpio)

            idRef = [0]
            cmdRef = [0]
            pSizeRef = [0]

            if self.receivePacketCommand(idRef, cmdRef, self.bufferPayload, pSizeRef, 1000, 50):
                self.localId = idRef[0]
                self.command = cmdRef[0]
                self.payloadSize = pSizeRef[0]

                if self.command == 0xC2 and self.payloadSize >= 5:
                    rawAnalog = (self.bufferPayload[3] << 8) | self.bufferPayload[4]
                    return rawAnalog

        return 0

    def digitalRead(self, dev_id, gpio):
        self.getGPIOStatus(dev_id, gpio)

        if self.analogEnabled and gpio > 4 and gpio < 7:
            v = self.analogRead(dev_id, gpio)
            if v >= (4096 // 2):
                return 1
            return 0

        idRef = [0]
        cmdRef = [0]
        pSizeRef = [0]

        if self.receivePacketCommand(idRef, cmdRef, self.bufferPayload, pSizeRef, 1000, 50):
            self.localId = idRef[0]
            self.command = cmdRef[0]
            self.payloadSize = pSizeRef[0]

            if self.command == 0xC2 and self.payloadSize >= 5:
                return self.bufferPayload[4]

        return 0

    def digitalWrite(self, dev_id, gpio, logical_level):
        if gpio > 0x07:
            return False
        if self.analogEnabled and gpio > 4 and gpio < 7:
            return False

        for _ in range(3):
            b = 0

            self.bufferPayload[b] = 0x01
            b += 1
            self.bufferPayload[b] = gpio
            b += 1
            self.bufferPayload[b] = logical_level
            b += 1
            self.bufferPayload[b] = 0x00

            if not self.prepareFrameCommand(dev_id & 0xFFFF, 0xC2, self.bufferPayload, b + 1):
                continue

            idRef = [0]
            cmdRef = [0]
            pSizeRef = [0]

            if self.receivePacketCommand(idRef, cmdRef, self.bufferPayload, pSizeRef, 1000, 50):
                self.localId = idRef[0]
                self.command = cmdRef[0]
                self.payloadSize = pSizeRef[0]

                if self.command == 0xC2 and self.payloadSize >= 4:
                    if self.bufferPayload[2] == gpio and self.bufferPayload[3] == logical_level:
                        return True

        return False

    def getNoise(self, dev_id, select=1):
        b = 0
        self.bufferPayload[b] = 0
        b += 1
        self.bufferPayload[b] = 0
        b += 1
        self.bufferPayload[b] = 0

        if select > 2:
            select = 2

        if not self.prepareFrameCommand(dev_id, 0xD8, self.bufferPayload, b + 1):
            return 255

        ridRef = [0]
        rcmdRef = [0]
        pszRef = [0]

        if not self.receivePacketCommand(ridRef, rcmdRef, self.bufferPayload, pszRef, 270, 50):
            return 255

        if rcmdRef[0] != 0xD8:
            return 255
        if pszRef[0] <= select:
            return 255

        return self.bufferPayload[select]

    def getR1(self, rawADC, R2):
        if rawADC == 0:
            return 0x7FFFFFFF
        if rawADC >= 4096:
            return 0
        return int((R2 * (4096 - rawADC)) / rawADC)

    def getTemp(self, rawADC, beta, Rt=10000, R2=10000):
        R1 = float(self.getR1(rawADC, R2))
        if R1 <= 0.0:
            return -273.15

        rx = float(Rt) * math.exp(-float(beta) / (273.0 + 25.0))
        t = float(beta) / math.log(R1 / rx)
        return t - 273.0