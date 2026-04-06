import utime as time
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


def drain_uart(uart):
    while uart.any():
        uart.read()


def initialize_radio(lora):
    print("Waiting for module...")

    while not lora.localRead():
        print("Module did not respond yet...")
        time.sleep_ms(500)

    print("Module ready")

    if lora.localId != ID:
        print("Configuring NetworkID")
        for _ in range(3):
            if lora.setNetworkID(ID):
                break
            time.sleep_ms(200)

    for _ in range(3):
        if lora.getBPS():
            break

    if lora.BW != BW500 or lora.SF != SF7 or lora.CR != CR4_5:
        print("Configuring BPS")
        for _ in range(3):
            if lora.setBPS(BW500, SF7, CR4_5):
                break
            time.sleep_ms(200)

    for _ in range(3):
        if lora.getClass():
            break

    if lora.LoRa_class != CLASS_C or lora.LoRa_window != WINDOW_5s:
        print("Configuring class")
        for _ in range(3):
            if lora.setClass(CLASS_C, WINDOW_5s):
                break
            time.sleep_ms(200)

    if lora.registered_password != PASSWORD:
        print("Configuring password")
        for _ in range(3):
            if lora.setPassword(PASSWORD):
                break
            time.sleep_ms(200)

    print("Configuration complete")


def main():
    print("Initializing LoRaMESH on ESP32...")
    time.sleep_ms(500)

    cmd_uart = esp32.create_cmd_uart()
    trp_uart = esp32.create_trp_uart()

    time.sleep_ms(300)

    drain_uart(cmd_uart)
    drain_uart(trp_uart)

    cmd_adapter = MachineUARTAdapter(uart=cmd_uart)
    trp_adapter = MachineUARTAdapter(uart=trp_uart)

    ticks = esp32.create_tick_source()

    cmd_transport = UARTTransport(cmd_adapter, ticks)
    trp_transport = UARTTransport(trp_adapter, ticks)

    lora = LoRaMESH(cmd_transport, trp_transport)

    lora.setDebug(debug_print, DEBUG)
    lora.begin(False)

    initialize_radio(lora)

    print("Master ready")

    state = False

    while True:
        state = not state

        if state:
            for i in range(1, MAX_REPLICATE_IN_SLAVES + 1):
                success = False

                for _ in range(MAX_RETRY):
                    if lora.digitalWrite(i, 0, 1):
                        success = True
                        break
                    time.sleep_ms(random_retry_delay())

                if success:
                    print("GPIO0 SLAVE:{}=ON".format(i))
                else:
                    print("GPIO0 SLAVE:{}=ON FAILED".format(i))

                if MAX_REPLICATE_IN_SLAVES > 1:
                    time.sleep_ms(DELAY_BETWEEN_SLAVES)

        else:
            for i in range(1, MAX_REPLICATE_IN_SLAVES + 1):
                success = False

                for _ in range(MAX_RETRY):
                    if lora.digitalWrite(i, 0, 0):
                        success = True
                        break
                    time.sleep_ms(random_retry_delay())

                if success:
                    print("GPIO0 SLAVE:{}=OFF".format(i))
                else:
                    print("GPIO0 SLAVE:{}=OFF FAILED".format(i))

                if MAX_REPLICATE_IN_SLAVES > 1:
                    time.sleep_ms(DELAY_BETWEEN_SLAVES)

        if MAX_REPLICATE_IN_SLAVES == 1:
            time.sleep_ms(DELAY_BETWEEN_SLAVES)


if __name__ == "__main__":
    main()