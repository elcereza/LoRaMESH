import sys
import time
import urandom

if "/sd" not in sys.path:
    sys.path.append("/sd")

from loramesh.boards import sipeed
from loramesh.adapters.maixpy_uart import MaixPyUARTAdapter
from loramesh.transport.UARTTransport import UARTTransport
from loramesh.core.LoRaMESH import LoRaMESH
from loramesh.constants import BW500, SF7, CR4_5, CLASS_C, WINDOW_5s

DEBUG = True

ID = 0
PASSWORD = 123

TARGET_ID = 1

RUN_FOREVER = True
TEST_ITERATIONS = 5


def debug_print(msg):
    print(msg)


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

    print("Configuring password")
    for _ in range(3):
        if mesh.setPassword(PASSWORD):
            print("Password configured")
            break
        time.sleep_ms(200)

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
    print("Initializing LoRaMESH on MaixBit...")

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

    mesh = LoRaMESH(cmd_transport, trp_transport)
    mesh.setDebug(debug_print, DEBUG)
    mesh.begin(DEBUG)

    initialize_radio(mesh)

    print("Command UART ready")
    print("Transparent UART ready")

    while True:
        if RUN_FOREVER:
            send_hello_world(mesh, TARGET_ID)
            time.sleep_ms(2000)
        else:
            for _ in range(TEST_ITERATIONS):
                send_hello_world(mesh, TARGET_ID)
                time.sleep_ms(2000)
            break


if __name__ == "__main__":
    main()