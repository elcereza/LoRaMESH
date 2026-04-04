# loramesh/boards/sipeed.py

UART_ID = 1
TX_PIN = 10
RX_PIN = 11
BAUD = 9600
UART_ADAPTER = "maixpy"
TICKS = "maix"

DATA_BITS = 8
PARITY = 0
STOP_BITS = 0
TIMEOUT = 1000
READ_BUF_LEN = 4096


def create_uart(
    uart_id=UART_ID,
    tx_pin=TX_PIN,
    rx_pin=RX_PIN,
    baud=BAUD,
    timeout=TIMEOUT,
    read_buf_len=READ_BUF_LEN
):
    from fpioa_manager import fm
    from machine import UART

    if uart_id == 1:
        fm.register(tx_pin, fm.fpioa.UART1_TX, force=True)
        fm.register(rx_pin, fm.fpioa.UART1_RX, force=True)
        hw_uart = UART.UART1

    elif uart_id == 2:
        fm.register(tx_pin, fm.fpioa.UART2_TX, force=True)
        fm.register(rx_pin, fm.fpioa.UART2_RX, force=True)
        hw_uart = UART.UART2

    elif uart_id == 3:
        fm.register(tx_pin, fm.fpioa.UART3_TX, force=True)
        fm.register(rx_pin, fm.fpioa.UART3_RX, force=True)
        hw_uart = UART.UART3

    else:
        raise ValueError("UART invalida para MaixPy: {}".format(uart_id))

    return UART(
        hw_uart,
        baud,
        DATA_BITS,
        PARITY,
        STOP_BITS,
        timeout=timeout,
        read_buf_len=read_buf_len
    )


def create_tick_source():
    from loramesh.backend.MaixTicks import MaixTicks
    return MaixTicks()