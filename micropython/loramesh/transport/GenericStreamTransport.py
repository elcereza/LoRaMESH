from .ILoRaTransport import ILoRaTransport


class GenericStreamTransport(ILoRaTransport):
    def __init__(self, stream, tick_source):
        self.stream = stream
        self.tick_source = tick_source

    def write(self, data, size):
        if data is None or size <= 0:
            return 0

        if len(data) < size:
            size = len(data)

        chunk = data[:size]
        n = self.stream.write(chunk)

        if n is None:
            return 0

        return int(n)

    def read(self, out, size):
        if out is None or size <= 0:
            return 0

        data = self.stream.read(size)
        if data is None:
            return 0

        n = len(data)
        if n > size:
            n = size

        for i in range(n):
            out[i] = data[i]

        return n

    def available(self):
        if hasattr(self.stream, "any"):
            v = self.stream.any()
            if v is None:
                return 0
            return int(v)

        if hasattr(self.stream, "in_waiting"):
            v = self.stream.in_waiting
            if callable(v):
                v = v()
            if v is None:
                return 0
            return int(v)

        if hasattr(self.stream, "available"):
            v = self.stream.available()
            if v is None:
                return 0
            return int(v)

        return 0

    def millis(self):
        return self.tick_source.millis()