UART_ID = 0
TX_PIN = 0
RX_PIN = 1
BAUD = 9600
UART_ADAPTER = "machine"
TICKS = "micropython"


def create_uart():
    from machine import UART

    return UART(
        UART_ID,
        baudrate=BAUD,
        tx=TX_PIN,
        rx=RX_PIN
    )