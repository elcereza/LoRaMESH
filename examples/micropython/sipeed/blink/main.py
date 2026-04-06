import sys
import os
import time
import urandom

# Ajusta os caminhos para buscar a biblioteca no SD
if "/sd" not in sys.path:
    sys.path.append("/sd")

if "/sd/" not in sys.path:
    sys.path.append("/sd/")

print("sys.path =", sys.path)

from loramesh.boards import sipeed
from loramesh.adapters.maixpy_uart import MaixPyUARTAdapter
from loramesh.transport.UARTTransport import UARTTransport
from loramesh.core.LoRaMESH import LoRaMESH
from loramesh.constants import BW500, SF7, CR4_5, CLASS_C, WINDOW_5s

DEBUG = True

MAX_REPLICATE_IN_SLAVES = 3
DELAY_BETWEEN_SLAVES = 250
MAX_RETRY = 3

ID = 0
PASSWORD = 123


def debug_print(msg):
    print(msg)


def random_retry_delay():
    return 1000 + (urandom.getrandbits(16) % 2000)


def drain_uart(uart):
    while uart.any():
        uart.read()


def initialize_radio(lora):
    print("Aguardando modulo...")

    while not lora.localRead():
        print("Modulo nao respondeu ainda...")
        time.sleep_ms(500)

    print("Modulo pronto")

    if lora.localId != ID:
        print("Configurando NetworkID")
        for _ in range(3):
            if lora.setNetworkID(ID):
                break
            time.sleep_ms(200)

    for _ in range(3):
        if lora.getBPS():
            break

    if lora.BW != BW500 or lora.SF != SF7 or lora.CR != CR4_5:
        print("Configurando BPS")
        for _ in range(3):
            if lora.setBPS(BW500, SF7, CR4_5):
                break
            time.sleep_ms(200)

    for _ in range(3):
        if lora.getClass():
            break

    if lora.LoRa_class != CLASS_C or lora.LoRa_window != WINDOW_5s:
        print("Configurando classe")
        for _ in range(3):
            if lora.setClass(CLASS_C, WINDOW_5s):
                break
            time.sleep_ms(200)

    if lora.registered_password != PASSWORD:
        print("Configurando senha")
        for _ in range(3):
            if lora.setPassword(PASSWORD):
                break
            time.sleep_ms(200)

    print("Configuracao concluida")


def main():
    print("Inicializando LoRaMesh no MaixBit...")

    cmd_uart = sipeed.create_cmd_uart()
    trp_uart = sipeed.create_trp_uart()

    time.sleep_ms(300)

    drain_uart(cmd_uart)
    drain_uart(trp_uart)

    cmd_adapter = MaixPyUARTAdapter(cmd_uart)
    trp_adapter = MaixPyUARTAdapter(trp_uart)

    ticks_cmd = sipeed.create_tick_source()
    ticks_trp = sipeed.create_tick_source()

    cmd_transport = UARTTransport(cmd_adapter, ticks_cmd)
    trp_transport = UARTTransport(trp_adapter, ticks_trp)

    lora = LoRaMESH(cmd_transport, trp_transport)
    lora.setDebug(debug_print, DEBUG)
    lora.begin(DEBUG)

    initialize_radio(lora)

    print("Master pronto")

    state = False

    while True:
        state = not state

        if state:
            for i in range(1, MAX_REPLICATE_IN_SLAVES + 1):
                for _ in range(MAX_RETRY):
                    if lora.digitalWrite(i, 0, 1):
                        break
                    time.sleep_ms(random_retry_delay())

                print("GPIO0 SLAVE:{}=ON".format(i))

                if MAX_REPLICATE_IN_SLAVES > 1:
                    time.sleep_ms(DELAY_BETWEEN_SLAVES)

        else:
            for i in range(1, MAX_REPLICATE_IN_SLAVES + 1):
                for _ in range(MAX_RETRY):
                    if lora.digitalWrite(i, 0, 0):
                        break
                    time.sleep_ms(random_retry_delay())

                print("GPIO0 SLAVE:{}=OFF".format(i))

                if MAX_REPLICATE_IN_SLAVES > 1:
                    time.sleep_ms(DELAY_BETWEEN_SLAVES)

        if MAX_REPLICATE_IN_SLAVES == 1:
            time.sleep_ms(DELAY_BETWEEN_SLAVES)


if __name__ == "__main__":
    main()