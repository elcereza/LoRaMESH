import loramesh
import serial
import time

CMD_PORT = "/dev/ttyUSB0"              #r"\\.\COM29"
TRANSPARENT_PORT = "/dev/ttyACM0"      #r"\\.\COM7"
BAUD = 9600

ID = 1
PASSWORD = 123

BW500 = 0x02
SF7 = 0x07
CR4_5 = 0x01

CLASS_C = 0x02
WINDOW_5s = 0x00


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


def read_transparent_packet(ser, first_timeout=1.0, inter_byte_timeout=0.03):
    start = time.time()
    packet = bytearray()

    while (time.time() - start) < first_timeout:
        if ser.in_waiting:
            packet.extend(ser.read(ser.in_waiting))
            break
        time.sleep(0.005)

    if not packet:
        return None

    last_rx = time.time()

    while True:
        if ser.in_waiting:
            packet.extend(ser.read(ser.in_waiting))
            last_rx = time.time()
        else:
            if (time.time() - last_rx) > inter_byte_timeout:
                break
            time.sleep(0.005)

    return bytes(packet)


def parse_transparent_packet(packet: bytes):
    if len(packet) < 2:
        raise ValueError("pacote transparente curto")

    source_id = packet[0] | (packet[1] << 8)
    payload = packet[2:]

    return source_id, payload


def main():
    print("Inicializando LoRaMesh receiver...")

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
    print("Aguardando pacotes...")

    try:
        while True:
            packet = read_transparent_packet(
                transparent_serial,
                first_timeout=0.2,
                inter_byte_timeout=0.03
            )

            if not packet:
                continue

            print(f"[RX transparente raw] {packet.hex(' ')}")

            try:
                source_id, payload = parse_transparent_packet(packet)

                try:
                    text = payload.decode("utf-8", errors="replace")
                except Exception:
                    text = repr(payload)

                print(f"[RX transparente] origem={source_id} payload={text!r}")

            except Exception as exc:
                print(f"Erro ao interpretar pacote: {exc}")

    except KeyboardInterrupt:
        print("\nEncerrando...")
    finally:
        transparent_serial.close()


if __name__ == "__main__":
    main()