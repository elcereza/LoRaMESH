import loramesh
import serial
import time

CMD_PORT = "/dev/ttyUSB0"              #r"\\.\COM29"
TRANSPARENT_PORT = "/dev/ttyACM0"      #r"\\.\COM7"
BAUD = 9600

ID = 0
PASSWORD = 123

BW500 = 0x02
SF7 = 0x07
CR4_5 = 0x01

CLASS_C = 0x02
WINDOW_5s = 0x00

TARGET_ID = 1


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
            time.sleep(0.2)

    for _ in range(3):
        if mesh.getClass():
            break

    if mesh.getClassValue() != CLASS_C or mesh.getWindow() != WINDOW_5s:
        print("Configurando classe")
        for _ in range(3):
            if mesh.setClass(CLASS_C, WINDOW_5s):
                break
            time.sleep(0.2)

    if mesh.setPassword(PASSWORD):
        print("Senha configurada")

    print("Configuracao concluida")


def build_transparent_frame(target_id, payload: bytes) -> bytes:
    if not (0 <= target_id <= 0xFFFF):
        raise ValueError("target_id fora do intervalo")

    return bytes([
        target_id & 0xFF,
        (target_id >> 8) & 0xFF
    ]) + payload


def send_hello_world(ser, target_id):
    payload = b"HelloWorld"
    frame = build_transparent_frame(target_id, payload)
    ser.write(frame)
    ser.flush()
    print(f"[TX transparente] destino={target_id} payload=HelloWorld hex={frame.hex(' ')}")


def main():
    print("Inicializando LoRaMesh...")

    mesh = loramesh.LoRaMESH(CMD_PORT, BAUD)
    mesh.begin(True, True)

    initialize_radio(mesh)

    print("Serial de comandos pronta")

    transparent_serial = serial.Serial(
        port=TRANSPARENT_PORT,
        baudrate=BAUD,
        timeout=0,
        write_timeout=1
    )

    transparent_serial.reset_input_buffer()
    transparent_serial.reset_output_buffer()

    print("Serial transparente pronta")

    try:
        while True:
            send_hello_world(transparent_serial, TARGET_ID)
            time.sleep(2)

    except KeyboardInterrupt:
        print("\nEncerrando...")
    finally:
        transparent_serial.close()


if __name__ == "__main__":
    main()