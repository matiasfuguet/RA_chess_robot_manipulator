import math
import socket
import time
import xml.etree.ElementTree as ET
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent  # repo_root/robot
REPO_ROOT = BASE_DIR.parent

HOST = "10.10.73.235"  # robot UR
PORT = 30002

ENABLE_PINZA = True

Abrir_pinza = BASE_DIR / "pinza40UR3.py"   # abre
Cerrar_pinza = BASE_DIR / "pinza10UR3.py"  # cierra sobre la pieza
TASKFILE = REPO_ROOT / "src/chess_manipulator/taskfile_chess_game_no_objects.xml"

# Los 6 joints del UR3 ocupan estas posiciones en cada Conf del taskfile.
UR3_JOINT_SLICE = slice(9, 15)

# HOME (= square_to_joints.HOME_JOINTS), duplicado aquí para no depender de nada
# fuera de la stdlib en el PC del robot. Solo se usa para marcar una pausa al
# pasar por HOME.
HOME_JOINTS_RAW = [d * math.pi / 180.0 for d in [79.77, -80.75, 55.31, -68.87, -87.87, 348.59]]


def _is_home(point, atol=1e-3):
    return all(abs(a - b) < atol for a, b in zip(point, HOME_JOINTS_RAW))


def load_paths_from_taskfile(taskfile):
    """taskfile_simplify.py (con checkpoints_only=True) ya divide cada accion
    real en su propio bloque, partiendo por cada parada con nombre (hover/
    home/agarre) - asi que cada bloque de aqui tiene exactamente los dos
    extremos de UN solo tramo. Por eso aqui basta con quedarse el primer y
    el ultimo punto de cada bloque: no hay ninguna parada intermedia que se
    pueda perder, porque ya no existen (estan en su propio bloque)."""
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

        #if len(points) > 2:
        #    points = [points[0], points[-1]]

        paths.append((block.tag, points))

    return paths


# pinza10UR3.py/pinza40UR3.py son exportaciones completas de PolyScope. Se
# mandan en crudo, byte a byte: cualquier intento de "limpiarlas" antes rompe
# la pinza (probado).
def send_script(filename, sock):
    with open(filename, "rb") as f:
        sock.sendall(f.read())
    time.sleep(1)


MOVEJ_ACCEL = 1.0  # subido de 0.5 ahora que el proceso completo va bien
MOVEJ_VEL = 1.0
MIN_POINT_WAIT = 0.5  # bajado de 1.0 ahora que el proceso completo va bien
HOME_DWELL_SECONDS = 1.5  # pausa extra al pasar por HOME


def _point_wait(prev, point, accel=MOVEJ_ACCEL, vel=MOVEJ_VEL):
    """Tiempo de espera del movej (perfil trapezoidal por el mayor delta
    articular), con MIN_POINT_WAIT como suelo."""
    if prev is None:
        return MIN_POINT_WAIT
    ramp_time = vel / accel
    delta = max(abs(a - b) for a, b in zip(point, prev))
    cruise = max(delta / vel - ramp_time, 0.0)
    return max(MIN_POINT_WAIT, (2 * ramp_time + cruise) * 1.3)


FIRST_MOVE_WAIT_SECONDS = 6.0  # cuanto tarde de verdad el robot en llegar a HOME
# desde donde estuviera antes de arrancar el script no lo sabemos (no hay
# feedback de posicion) - _point_wait con prev=None solo da el suelo (0.5s),
# que vale para "ya estaba ahi" (todo bloque salvo el primero arranca en el
# punto donde acabo el anterior) pero no para este primer movimiento, de
# distancia desconocida. Una espera fija y generosa solo para este caso.
_first_movej_sent = False


def send_joint_path(path, sock):
    """Un movej suelto por punto, esperando siempre a que termine antes de
    mandar el siguiente. Probado con radio de mezcla (sin esperar el tiempo
    real de cada tramo intermedio) y se perdia la sincronia entre lo que el
    script asume y donde esta realmente el robot - acababa mandando la accion
    de la pinza antes de llegar de verdad. Vuelta a la espera completa."""
    global _first_movej_sent
    prev = None
    for joint_config in path:
        print(f"  -> movej {joint_config}")
        sock.send(f"movej({joint_config}, a={MOVEJ_ACCEL}, v={MOVEJ_VEL})\n".encode())

        if not _first_movej_sent:
            wait = FIRST_MOVE_WAIT_SECONDS
            print(f"  -> Primer movimiento del script, pausa de {wait}s")
            _first_movej_sent = True
        else:
            wait = _point_wait(prev, joint_config)
            if _is_home(joint_config):
                print(f"  -> En HOME, pausa de {HOME_DWELL_SECONDS}s")
                wait += HOME_DWELL_SECONDS
        time.sleep(wait)
        prev = joint_config


PLANS_DIR = REPO_ROOT / "plans"
PREVIEW = PLANS_DIR / "robot_plan_preview.txt"


def build_action_sequence(paths):
    """Secuencia ordenada de acciones: ('move', kind, idx, puntos) y
    ('pinza', 'abrir'/'cerrar'). Una sola fuente para la previsualización y
    la ejecución, así no se pueden desincronizar.

    Desde que taskfile_simplify.py parte cada transporte en un bloque por
    tramo (uno por cada hover intermedio), un mismo "transporte" puede ser
    VARIOS bloques Transfer consecutivos, no uno solo - así que solo se abre
    la pinza despues del ULTIMO Transfer de la racha (cuando el bloque
    siguiente ya no es otro Transfer), no despues de cada uno."""
    actions = []
    if ENABLE_PINZA:
        actions.append(("pinza", "abrir"))  # estado inicial
    for idx, (kind, path) in enumerate(paths):
        actions.append(("move", kind, idx, path))
        if not ENABLE_PINZA:
            continue
        if kind == "Transit" and idx + 1 < len(paths) and paths[idx + 1][0] == "Transfer":
            actions.append(("pinza", "cerrar"))
        elif kind == "Transfer" and (idx + 1 >= len(paths) or paths[idx + 1][0] != "Transfer"):
            actions.append(("pinza", "abrir"))
    return actions


def write_preview(actions, path):
    """Vuelca lo que el robot va a hacer (cada movej y cada apertura/cierre de
    pinza) para poder revisarlo antes de moverlo. No conecta ni mueve nada."""
    path.parent.mkdir(exist_ok=True)
    with open(path, "w") as f:
        f.write("# Previsualizacion - el robot NO se ha movido todavia.\n")
        f.write(f"# Generado desde {TASKFILE.name}\n\n")
        for action in actions:
            if action[0] == "pinza":
                f.write(f"PINZA: {action[1].upper()}\n")
            else:
                _, kind, idx, points = action
                f.write(f"[{idx}] {kind} ({len(points)} puntos)\n")
                for point in points:
                    f.write(f"    movej {point}\n")


paths = load_paths_from_taskfile(TASKFILE)
if not paths:
    raise RuntimeError(
        f"No hay bloques Transit/Transfer con Conf en {TASKFILE}. "
        "Genera primero el taskfile con Kautham/TAMP."
    )

actions = build_action_sequence(paths)
write_preview(actions, PREVIEW)
print(f"Previsualizacion guardada en {PREVIEW}")
print(f"Cargados {len(paths)} bloques de trayectoria desde {TASKFILE}")

# Conexión via socket a la controladora del robot
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((HOST, PORT))

for action in actions:
    if action[0] == "pinza":
        print("Abriendo pinza..." if action[1] == "abrir" else "Cerrando pinza...")
        send_script(Abrir_pinza if action[1] == "abrir" else Cerrar_pinza, sock)
    else:
        _, kind, idx, path = action
        print(f"Ejecutando {kind} {idx}...")
        send_joint_path(path, sock)

print("Trayectoria finalizada")

data = sock.recv(1024)
sock.close()
