class GenericUARTAdapter:
    def __init__(self, uart):
        if uart is None:
            raise ValueError("uart cannot be None")
        self.uart = uart

    def write(self, data):
        if data is None:
            return 0

        n = self.uart.write(data)
        if n is None:
            return 0

        return int(n)

    def read(self, size=1):
        if size is None or size <= 0:
            return b""

        try:
            data = self.uart.read(size)
        except TypeError:
            data = self.uart.read()

        if data is None:
            return b""

        if isinstance(data, int):
            return bytes((data & 0xFF,))

        if isinstance(data, str):
            return data.encode()

        try:
            return bytes(data)
        except TypeError:
            return b""

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

    def available(self):
        return self.any()