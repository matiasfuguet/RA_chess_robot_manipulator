import socket
import time
import xml.etree.ElementTree as ET
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent

# Dirección IP del robot UR
HOST = "10.10.73.234"

# Puerto del servidor en el robot
PORT = 30002

# Scripts para abrir y cerrar la pinza
Abrir_pinza = BASE_DIR / "abrir_pinza.py"
Cerrar_pinza = BASE_DIR / "cerrar_pinza.py"
TASKFILE = BASE_DIR / "src/chess_manipulator/taskfile_tampconfig_chess_real.xml"

# En el taskfile de Kautham, los 6 joints del UR3 salen en estas posiciones.
UR3_JOINT_SLICE = slice(9, 15)


def load_paths_from_taskfile(taskfile):
    tree = ET.parse(taskfile)
    root = tree.getroot()
    paths = []

    for block in root:
        if block.tag not in ("Transit", "Transfer"):
            continue

        points = []
        for conf in block.findall("Conf"):
            values = [float(x) for x in conf.text.split()]
            points.append(values[UR3_JOINT_SLICE])

        if points:
            paths.append((block.tag, points))

    return paths


# Función para enviar una trayectoria en espacio de configuraciones a la controladora del robot
def send_joint_path(path, sock):
    for joint_config in path:
        print(joint_config)
        sock.send(f"movej({joint_config}, a=0.5, v=0.5)".encode() + b"\n")
        time.sleep(1)


def send_script(filename, sock):
    with open(filename, "rb") as f:
        sock.sendall(f.read())
    time.sleep(3)


paths = load_paths_from_taskfile(TASKFILE)
if not paths:
    raise RuntimeError(
        f"No hay bloques Transit/Transfer con Conf en {TASKFILE}. "
        "Genera primero el taskfile con Kautham/TAMP."
    )
print(f"Cargados {len(paths)} bloques de trayectoria desde {TASKFILE}")

# Conexión via socket a la controladora del robot
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((HOST, PORT))

for idx, (kind, path) in enumerate(paths):
    print(f"Ejecutando {kind} {idx}...")
    send_joint_path(path, sock)

    if kind == "Transit" and idx + 1 < len(paths) and paths[idx + 1][0] == "Transfer":
        print("Cerrando pinza...")
        send_script(Cerrar_pinza, sock)
    elif kind == "Transfer":
        print("Abriendo pinza...")
        send_script(Abrir_pinza, sock)

print("Trayectoria finalizada")

data = sock.recv(1024)
sock.close()