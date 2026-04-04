from .GenericStreamTransport import GenericStreamTransport


class UARTTransport(GenericStreamTransport):
    def __init__(self, uart, tick_source):
        super().__init__(uart, tick_source)