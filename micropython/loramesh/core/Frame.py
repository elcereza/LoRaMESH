from ..constants import MAX_BUFFER_SIZE


class Frame:
    def __init__(self):
        self.buffer = bytearray(MAX_BUFFER_SIZE)
        self.size = 0
        self.command = True  # True = cmd transport, False = transparent transport