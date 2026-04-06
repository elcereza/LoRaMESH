CMD_UART_ID = 1
CMD_TX_PIN = 10
CMD_RX_PIN = 11

TRP_UART_ID = 2
TRP_TX_PIN = 8
TRP_RX_PIN = 9

BAUD = 9600
UART_ADAPTER = "maixpy"
TICKS = "maix"

DATA_BITS = 8
PARITY = 0
STOP_BITS = 0
TIMEOUT = 1000
READ_BUF_LEN = 4096


def _resolve_uart_and_register_pins(uart_id, tx_pin, rx_pin):
    from fpioa_manager import fm
    from machine import UART

    if uart_id == 1:
        fm.register(tx_pin, fm.fpioa.UART1_TX, force=True)
        fm.register(rx_pin, fm.fpioa.UART1_RX, force=True)
        return UART.UART1

    if uart_id == 2:
        fm.register(tx_pin, fm.fpioa.UART2_TX, force=True)
        fm.register(rx_pin, fm.fpioa.UART2_RX, force=True)
        return UART.UART2

    if uart_id == 3:
        fm.register(tx_pin, fm.fpioa.UART3_TX, force=True)
        fm.register(rx_pin, fm.fpioa.UART3_RX, force=True)
        return UART.UART3

    raise ValueError("Invalid MaixPy UART id: {}".format(uart_id))


def create_uart(
    uart_id,
    tx_pin,
    rx_pin,
    baud=BAUD,
    timeout=TIMEOUT,
    read_buf_len=READ_BUF_LEN,
    data_bits=DATA_BITS,
    parity=PARITY,
    stop_bits=STOP_BITS
):
    from machine import UART

    hw_uart = _resolve_uart_and_register_pins(uart_id, tx_pin, rx_pin)

    return UART(
        hw_uart,
        baud,
        data_bits,
        parity,
        stop_bits,
        timeout=timeout,
        read_buf_len=read_buf_len
    )


def create_cmd_uart():
    return create_uart(
        uart_id=CMD_UART_ID,
        tx_pin=CMD_TX_PIN,
        rx_pin=CMD_RX_PIN
    )


def create_trp_uart():
    return create_uart(
        uart_id=TRP_UART_ID,
        tx_pin=TRP_TX_PIN,
        rx_pin=TRP_RX_PIN
    )


def create_tick_source():
    from loramesh.backend.MaixTicks import MaixTicks
    return MaixTicks()