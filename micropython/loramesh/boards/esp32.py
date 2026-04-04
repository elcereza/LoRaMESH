UART_ID = 1
TX_PIN = 17
RX_PIN = 16
BAUD = 9600
UART_ADAPTER = "machine"
TICKS = "micropython"


def create_uart(
    uart_id=UART_ID,
    tx_pin=TX_PIN,
    rx_pin=RX_PIN,
    baud=BAUD
):
    from machine import UART

    return UART(
        uart_id,
        baudrate=baud,
        tx=tx_pin,
        rx=rx_pin
    )


def create_tick_source():
    from loramesh.backend.MicroPythonTicks import MicroPythonTicks
    return MicroPythonTicks()