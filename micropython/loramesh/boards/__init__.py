# loramesh/boards/__init__.py

from . import esp32
from . import rp2
from . import sipeed

def get_board(name):
    name = name.lower()

    if name == "esp32":
        return esp32
    if name == "rp2":
        return rp2
    if name == "sipeed":
        return sipeed

    raise ValueError("Board nao suportada: {}".format(name))