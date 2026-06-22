import json
import math
import socket
import time
import xml.etree.ElementTree as ET
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent

# Dirección IP del robot UR
HOST = "10.10.73.234"

# Puerto del servidor en el robot
PORT = 30002

ENABLE_PINZA = True

# Scripts para abrir y cerrar la pinza - anchura mayor (40mm) = abierta,
# menor (10mm) = cerrada sobre la pieza.
Abrir_pinza = BASE_DIR / "pinza40UR3.py"
Cerrar_pinza = BASE_DIR / "pinza10UR3.py"
TASKFILE = BASE_DIR / "src/chess_manipulator/taskfile_chess_game_no_objects.xml"

# Manifesto generado por run_game.py junto al taskfile (mismo nombre, sufijo
# "_hover"): lista de {"square": [...6 joints...], "hover": [...6 joints...]}
# por cada casilla realmente usada en la partida. PICK/PLACE (en
# ktmpb_client, codigo compartido del lab, no de este repo) resuelven su
# propio tramo home<->agarre internamente, y una vez el robot ya lleva una
# pieza agarrada, ese tramo interno puede ir derecho sin pasar por el hover
# - aqui se usa el manifesto para reinsertar esa parada por valor de
# articulaciones, sin depender de como haya resuelto el camino ktmpb.
HOVER_MANIFEST = BASE_DIR / "src/chess_manipulator/taskfile_chess_game_no_objects_hover.json"

# En el taskfile de Kautham, los 6 joints del UR3 salen en estas posiciones.
UR3_JOINT_SLICE = slice(9, 15)

# La config HOME usada por run_game.py/square_to_joints.py (HOME_JOINTS) -
# duplicada aquí en vez de importar square_to_joints.py, para que este script
# siga sin dependencias en el PC del robot (solo stdlib). Sirve solo para dar
# una pausa visible al pasar por HOME - cada movej ya para por completo
# (ver send_joint_path), así que no hace falta nada más para que el robot
# realmente se detenga ahí.
#
# HOME ya no es D5_SEED_JOINTS (eso quedaba a la altura de agarre de d5,
# pegado al tablero, sin margen real con lo que hubiera en esa casilla) -
# ahora es una config articular ensenada directamente en el robot real,
# bien por encima del tablero (ver "home" en posiciones_reales.md y
# HOME_JOINTS_DEG en square_to_joints.py).
HOME_JOINTS_RAW = [d * math.pi / 180.0 for d in [79.77, -80.75, 55.31, -68.87, -87.87, 348.59]]


def _is_home(point, atol=1e-3):
    return all(abs(a - b) < atol for a, b in zip(point, HOME_JOINTS_RAW))


def load_paths_from_taskfile(taskfile, step=20):
    """
    step=20 significa que cogerá 1 de cada 20 puntos de Kautham.
    Si ves que choca, baja el número. Si quieres que vaya más recto, súbelo.
    """
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

        if not points:
            continue

        # OPTIMIZACIÓN: Cogemos puntos de 'step' en 'step'
        simplified_points = points[::step]

        # Nos aseguramos SIEMPRE de incluir la posición final exacta
        if simplified_points[-1] != points[-1]:
            simplified_points.append(points[-1])

        paths.append((block.tag, simplified_points))

    return paths


def _load_hover_manifest(path):
    if not path.exists():
        print(f"Aviso: no encuentro {path} - no puedo garantizar la parada en hover.")
        return []
    with open(path) as f:
        return json.load(f)


def _close_enough(a, b, atol=1e-3):
    return all(abs(x - y) < atol for x, y in zip(a, b))


def _ensure_hover_before_squares(paths, manifest):
    """Inserta un movej explicito al hover justo antes de llegar a
    cualquier casilla conocida (por el manifesto), si el punto
    inmediatamente anterior no es ya ese hover. Necesario porque PICK/PLACE
    (ktmpb_client, no es codigo de este repo) resuelven su propio tramo
    home<->agarre por su cuenta, y una vez la pieza ya esta agarrada ese
    tramo puede ir derecho a la siguiente casilla sin pasar por el hover -
    aqui se garantiza la parada por valor de articulaciones, sin depender
    de por donde haya resuelto el camino ktmpb."""
    if not manifest:
        return paths

    prev = None
    new_paths = []
    for kind, points in paths:
        new_points = []
        for point in points:
            match = next((m for m in manifest if _close_enough(point, m["square"])), None)
            if match and not (prev is not None and _close_enough(prev, match["hover"])):
                new_points.append(match["hover"])
            new_points.append(point)
            prev = point
        new_paths.append((kind, new_points))

    return new_paths


# Los ficheros exportados de pinza (pinza10UR3.py/pinza40UR3.py) son
# exportaciones completas de PolyScope, con su "def NOMBRE(): ... end" y el
# bucle infinito del nodo de programa intactos. Probado aparte por el
# usuario: mandar el fichero tal cual, en crudo, por el socket SI funciona
# - cualquier intento de "limpiarlo" antes de mandarlo (quitar el bucle,
# extraer solo la llamada a RG2...) es lo que rompia la pinza. Asi que
# aqui no se toca nada, solo se reenvia el fichero byte a byte.
def send_script(filename, sock):
    with open(filename, "rb") as f:
        sock.sendall(f.read())
    time.sleep(1)


MOVEJ_ACCEL = 0.5
MOVEJ_VEL = 0.5
MIN_POINT_WAIT = 1.0  # igual que mover_robot.py (probado que funciona)
HOME_DWELL_SECONDS = 1.5  # pausa extra y visible al pasar por HOME


def _point_wait(prev, point, accel=MOVEJ_ACCEL, vel=MOVEJ_VEL):
    """MIN_POINT_WAIT como suelo (igual que mover_robot.py), con margen
    extra para movimientos largos via el mayor delta articular."""
    if prev is None:
        return MIN_POINT_WAIT
    ramp_time = vel / accel
    delta = max(abs(a - b) for a, b in zip(point, prev))
    cruise = max(delta / vel - ramp_time, 0.0)
    return max(MIN_POINT_WAIT, (2 * ramp_time + cruise) * 1.3)


def send_joint_path(path, sock):
    """Misma estructura que mover_robot.py: cada punto se envia como una
    instruccion movej SUELTA (sin envolver en un "def trayectoria(): ...
    end" que nunca se llamaba - ese era el bug real por el que el robot no
    se movia bien). El controlador ejecuta cada movej de forma bloqueante
    antes de pasar al siguiente, asi que no hay riesgo de pisar un
    movimiento a medias."""
    prev = None
    for joint_config in path:
        print(f"  -> movej {joint_config}")
        sock.send(f"movej({joint_config}, a={MOVEJ_ACCEL}, v={MOVEJ_VEL})\n".encode())

        wait = _point_wait(prev, joint_config)
        if _is_home(joint_config):
            print(f"  -> En HOME, pausa de {HOME_DWELL_SECONDS}s")
            wait += HOME_DWELL_SECONDS
        time.sleep(wait)
        prev = joint_config


paths = load_paths_from_taskfile(TASKFILE)
if not paths:
    raise RuntimeError(
        f"No hay bloques Transit/Transfer con Conf en {TASKFILE}. "
        "Genera primero el taskfile con Kautham/TAMP."
    )
paths = _ensure_hover_before_squares(paths, _load_hover_manifest(HOVER_MANIFEST))
print(f"Cargados {len(paths)} bloques de trayectoria desde {TASKFILE}")

# Conexión via socket a la controladora del robot
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((HOST, PORT))

if ENABLE_PINZA:
    print("Abriendo pinza (estado inicial)...")
    send_script(Abrir_pinza, sock)

for idx, (kind, path) in enumerate(paths):
    print(f"Ejecutando {kind} {idx}...")
    send_joint_path(path, sock)

    if not ENABLE_PINZA:
        continue

    if kind == "Transit" and idx + 1 < len(paths) and paths[idx + 1][0] == "Transfer":
        print("Cerrando pinza...")
        send_script(Cerrar_pinza, sock)
    elif kind == "Transfer":
        print("Abriendo pinza...")
        send_script(Abrir_pinza, sock)

print("Trayectoria finalizada")

data = sock.recv(1024)
sock.close()
