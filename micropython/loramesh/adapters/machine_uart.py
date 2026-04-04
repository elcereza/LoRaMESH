from .generic_uart import GenericUARTAdapter


class MachineUARTAdapter(GenericUARTAdapter):
    def __init__(self, uart=None, uart_id=None, baudrate=9600, tx=None, rx=None, **kwargs):
        if uart is None:
            from machine import UART

            if uart_id is None:
                raise ValueError("uart_id is required when uart is not provided")

            if tx is None and rx is None:
                uart = UART(uart_id, baudrate=baudrate, **kwargs)
            else:
                uart = UART(uart_id, baudrate=baudrate, tx=tx, rx=rx, **kwargs)

        super().__init__(uart)