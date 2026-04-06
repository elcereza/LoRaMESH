from machine import UART, Pin
import utime as time

from loramesh.core.LoRaMESH import LoRaMESH
from loramesh.adapters.machine_uart import MachineUARTAdapter
from loramesh.transport.UARTTransport import UARTTransport
from loramesh.backend.MicroPythonTicks import MicroPythonTicks

CMD_UART_ID = 0
CMD_TX_PIN = 0
CMD_RX_PIN = 1

TRP_UART_ID = 1
TRP_TX_PIN = 4
TRP_RX_PIN = 5

BAUD = 9600

ID = 0
PASSWORD = 123

BW500 = 0x02
SF7 = 0x07
CR4_5 = 0x01

CLASS_C = 0x02
WINDOW_5s = 0x00

TARGET_ID = 1

RUN_FOREVER = True
TEST_ITERATIONS = 5


def debug_print(msg):
    print(msg)


def create_uart(uart_id, tx_pin, rx_pin, baud=BAUD):
    return UART(
        uart_id,
        baudrate=baud,
        tx=Pin(tx_pin),
        rx=Pin(rx_pin),
        timeout=0
    )


def drain_uart(uart):
    while uart.any():
        uart.read()


def initialize_radio(mesh):
    print("Waiting for module...")

    while not mesh.localRead():
        print("Module did not respond yet...")
        time.sleep_ms(500)

    print("Module ready")

    if mesh.localId != ID:
        print("Configuring NetworkID")
        for _ in range(3):
            if mesh.setNetworkID(ID):
                break
            time.sleep_ms(200)

    for _ in range(3):
        if mesh.getBPS():
            break

    if mesh.BW != BW500 or mesh.SF != SF7 or mesh.CR != CR4_5:
        print("Configuring BPS")
        for _ in range(3):
            if mesh.setBPS(BW500, SF7, CR4_5):
                break
            time.sleep_ms(200)

    for _ in range(3):
        if mesh.getClass():
            break

    if mesh.LoRa_class != CLASS_C or mesh.LoRa_window != WINDOW_5s:
        print("Configuring class")
        for _ in range(3):
            if mesh.setClass(CLASS_C, WINDOW_5s):
                break
            time.sleep_ms(200)

    if mesh.setPassword(PASSWORD):
        print("Password configured")

    print("Configuration complete")


def send_hello_world(mesh, target_id):
    payload = b"HelloWorld"

    if not mesh.PrepareFrameTransp(target_id, payload, len(payload)):
        print("Failed to build transparent frame")
        return False

    if not mesh.sendPacket():
        print("Failed to send transparent frame")
        return False

    frame = bytes((
        target_id & 0xFF,
        (target_id >> 8) & 0xFF
    )) + payload

    print("[TX transparent] target={} payload=HelloWorld bytes={} hex={}".format(
        target_id,
        len(frame),
        " ".join("{:02x}".format(b) for b in frame)
    ))
    return True


def main():
    print("Initializing LoRaMESH on RP2040...")

    cmd_uart = create_uart(CMD_UART_ID, CMD_TX_PIN, CMD_RX_PIN)
    trp_uart = create_uart(TRP_UART_ID, TRP_TX_PIN, TRP_RX_PIN)

    time.sleep_ms(300)

    drain_uart(cmd_uart)
    drain_uart(trp_uart)

    cmd_adapter = MachineUARTAdapter(uart=cmd_uart)
    trp_adapter = MachineUARTAdapter(uart=trp_uart)

    ticks = MicroPythonTicks()

    cmd_transport = UARTTransport(cmd_adapter, ticks)
    trp_transport = UARTTransport(trp_adapter, ticks)

    mesh = LoRaMESH(cmd_transport, trp_transport)
    mesh.setDebug(debug_print, True)

    initialize_radio(mesh)

    print("Command UART ready")
    print("Transparent UART ready")

    try:
        if RUN_FOREVER:
            while True:
                send_hello_world(mesh, TARGET_ID)
                time.sleep_ms(2000)
        else:
            for _ in range(TEST_ITERATIONS):
                send_hello_world(mesh, TARGET_ID)
                time.sleep_ms(2000)

    except KeyboardInterrupt:
        print("Stopping...")


if __name__ == "__main__":
    main()