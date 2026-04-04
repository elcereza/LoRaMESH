from .generic_uart import GenericUARTAdapter


class MaixPyUARTAdapter(GenericUARTAdapter):
    def __init__(self, uart):
        super().__init__(uart)

    def any(self):
        if hasattr(self.uart, "any"):
            v = self.uart.any()
            if v is None:
                return 0
            return int(v)

        if hasattr(self.uart, "in_waiting"):
            v = self.uart.in_waiting
            if callable(v):
                v = v()
            if v is None:
                return 0
            return int(v)

        if hasattr(self.uart, "available"):
            v = self.uart.available()
            if v is None:
                return 0
            return int(v)

        return 0