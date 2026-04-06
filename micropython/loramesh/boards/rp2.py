CMD_UART_ID = 0
CMD_TX_PIN = 0
CMD_RX_PIN = 1

TRP_UART_ID = 1
TRP_TX_PIN = 4
TRP_RX_PIN = 5

BAUD = 9600
UART_ADAPTER = "machine"
TICKS = "micropython"


def create_uart(uart_id, tx_pin, rx_pin, baud=BAUD):
    from machine import UART, Pin

    return UART(
        uart_id,
        baudrate=baud,
        tx=Pin(tx_pin),
        rx=Pin(rx_pin),
        timeout=0
    )


def create_cmd_uart():
    return create_uart(CMD_UART_ID, CMD_TX_PIN, CMD_RX_PIN)


def create_trp_uart():
    return create_uart(TRP_UART_ID, TRP_TX_PIN, TRP_RX_PIN)


def create_tick_source():
    from loramesh.backend.MicroPythonTicks import MicroPythonTicks
    return MicroPythonTicks()