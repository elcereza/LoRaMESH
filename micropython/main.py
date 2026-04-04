import time
import urandom

from loramesh.boards import esp32
from loramesh.adapters.machine_uart import MachineUARTAdapter
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
    return 1500 + (urandom.getrandbits(16) % 2000)


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

    for _ in range(3):
        if lora.getClass():
            break

    if lora.LoRa_class != CLASS_C or lora.LoRa_window != WINDOW_5s:
        print("Configurando classe")
        for _ in range(3):
            if lora.setClass(CLASS_C, WINDOW_5s):
                break

    if lora.registered_password != PASSWORD:
        print("Configurando senha")
        for _ in range(3):
            if lora.setPassword(PASSWORD):
                break

    print("Configuracao concluida")


print("Inicializando LoRaMesh...")
time.sleep(2)

uart = esp32.create_uart()
adapter = MachineUARTAdapter(uart)
ticks = esp32.create_tick_source()
transport = UARTTransport(adapter, ticks)

lora = LoRaMESH(transport, transport)

lora.setDebug(debug_print, DEBUG)
lora.begin(False)

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