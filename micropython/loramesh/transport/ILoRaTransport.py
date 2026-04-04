class ILoRaTransport:
    def write(self, data, size):
        raise NotImplementedError

    def read(self, out, size):
        raise NotImplementedError

    def available(self):
        raise NotImplementedError

    def millis(self):
        raise NotImplementedError