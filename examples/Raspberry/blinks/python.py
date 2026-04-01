import loramesh
import time
import random

PORT = r"\\.\COM29"
BAUD = 115200

ID = 0

MAX_REPLICATE_IN_SLAVES = 3
DELAY_BETWEEN_SLAVES = 0.250
MAX_RETRY = 3

BW500 = 0x02
SF7 = 0x07
CR4_5 = 0x01

CLASS_C = 0x02
WINDOW_5s = 0x00

PASSWORD = 123

def initialize_radio(mesh):
    print("Aguardando modulo...")

    while not mesh.localRead():
        print("Modulo nao respondeu ainda...")
        time.sleep(0.5)

    print("Modulo pronto")
    if mesh.getLocalID() != ID:
        print("Configurando NetworkID")
        for _ in range(3):
            if mesh.setNetworkID(ID):
                break
            time.sleep(0.2)

    for _ in range(3):
        if mesh.getBPS():
            break
    if mesh.getBW() != BW500 or mesh.getSF() != SF7 or mesh.getCR() != CR4_5:
        print("Configurando BPS")
        for _ in range(3):
            if mesh.setBPS(BW500, SF7, CR4_5):
                break

    for _ in range(3):
        if mesh.getClass():
            break

    if mesh.getClassValue() != CLASS_C or mesh.getWindow() != WINDOW_5s:
        print("Configurando classe")
        for _ in range(3):
            if mesh.setClass(CLASS_C, WINDOW_5s):
                break

    if mesh.setPassword(PASSWORD):
        print("Senha configurada")
    print("Configuracao concluida")


def main():
    print("Inicializando LoRaMesh...")
    mesh = loramesh.LoRaMESH(PORT, BAUD)
    mesh.begin(True, True)
    initialize_radio(mesh)
    print("Master pronto")
    state = False

    while True:
        state = not state
        for i in range(1, MAX_REPLICATE_IN_SLAVES + 1):
            for j in range(MAX_RETRY):
                if mesh.digitalWrite(i, 0, 1 if state else 0):
                    break
                else:
                    delay = 1 + random.uniform(0.5, 2.5)
                    time.sleep(delay)

            if state:
                print(f"GPIO0 SLAVE:{i}=ON")
            else:
                print(f"GPIO0 SLAVE:{i}=OFF")
            if MAX_REPLICATE_IN_SLAVES > 1:
                time.sleep(DELAY_BETWEEN_SLAVES)

        if MAX_REPLICATE_IN_SLAVES == 1:
            time.sleep(DELAY_BETWEEN_SLAVES)


if __name__ == "__main__":
    main()
