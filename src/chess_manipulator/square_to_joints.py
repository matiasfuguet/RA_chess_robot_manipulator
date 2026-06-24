"""Pasa de una casilla del tablero (o slot del graveyard) a joints del UR3e
y al XML que pide ktmpb.

La geometria esta calibrada con los puntos  que hay en
docs/posiciones_reales.md. Solo dejamos activas las filas/slots que de verdad
convergen bien en las pruebas, las demas dan problemas de IK.
"""

import numpy as np

# --- cinematica ---

SEGMENTS = [
    ((0, 0, 0.15185), (0, 0, 0)),
    ((0, 0, 0), (1.570796327, 0, 0)),
    ((-0.24355, 0, 0), (0, 0, 0)),
    ((-0.2132, 0, 0.13105), (0, 0, 0)),
    ((0, -0.08535, -1.750557762378351e-11), (1.570796327, 0, 0)),
    ((0, 0.0921, -1.8890025766262e-11), (1.570796326589793, 3.141592653589793, 3.141592653589793)),
]
TCP_OFFSET = np.array([0.0, 0.0, 0.2286])  # punta de la pinza, a lo largo del eje Z de la herramienta
BASE_OFFSET = np.array([-0.015467, 0.013733, -0.347767])  # ajustado a partir de los puntos enseñados


def _rot_x(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _rot_y(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _segment_transform(xyz, rpy, joint_angle):
    roll, pitch, yaw = rpy
    t = np.eye(4)
    t[:3, :3] = _rot_z(yaw) @ _rot_y(pitch) @ _rot_x(roll) @ _rot_z(joint_angle)
    t[:3, 3] = xyz
    return t


def _rotmat_to_rotvec(r):
    theta = np.arccos(np.clip((np.trace(r) - 1) / 2, -1.0, 1.0))
    if theta < 1e-8:
        return np.zeros(3)
    axis = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]]) / (2 * np.sin(theta))
    return axis * theta


def _rotvec_to_rotmat(rv):
    theta = np.linalg.norm(rv)
    if theta < 1e-8:
        return np.eye(3)
    kx, ky, kz = rv / theta
    kmat = np.array([[0, -kz, ky], [kz, 0, -kx], [-ky, kx, 0]])
    return np.eye(3) + np.sin(theta) * kmat + (1 - np.cos(theta)) * (kmat @ kmat)


def forward_kinematics(joints):
    """6 angulos (rad) -> pose (x, y, z, rx, ry, rz) de la pinza, en el frame del robot."""
    t = np.eye(4)
    for (xyz, rpy), q in zip(SEGMENTS, joints):
        t = t @ _segment_transform(xyz, rpy, q)
    t[:3, 3] = t[:3, 3] + t[:3, :3] @ TCP_OFFSET
    return np.concatenate([t[:3, 3] + BASE_OFFSET, _rotmat_to_rotvec(t[:3, :3])])


def _ik_attempt(target_pose, seed_joints, max_iters, tol):
    target_pos, target_rot = np.array(target_pose[:3]), _rotvec_to_rotmat(np.array(target_pose[3:]))
    q = np.array(seed_joints, dtype=float)
    eps, damping = 1e-6, 1e-3

    for _ in range(max_iters):
        pose = forward_kinematics(q)
        cur_pos, cur_rot = pose[:3], _rotvec_to_rotmat(pose[3:])
        err = np.concatenate([target_pos - cur_pos, _rotmat_to_rotvec(target_rot @ cur_rot.T)])
        if np.linalg.norm(err) < tol:
            return q, True

        # jacobiano numerico, columna a columna
        jacobian = np.zeros((6, 6))
        for i in range(6):
            dq = q.copy()
            dq[i] += eps
            pert = forward_kinematics(dq)
            d_pos = (pert[:3] - cur_pos) / eps
            d_rot = _rotmat_to_rotvec(_rotvec_to_rotmat(pert[3:]) @ cur_rot.T) / eps
            jacobian[:, i] = np.concatenate([d_pos, d_rot])

        dq = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + damping * np.eye(6), err)
        q = q + dq

    return q, False


def inverse_kinematics(target_pose, seed_joints, max_iters=200, tol=1e-8):
    """IK por minimos cuadrados amortiguados, con seed cerca para caer en la misma rama que el robot real."""
    q, ok = _ik_attempt(target_pose, seed_joints, max_iters, tol)
    if not ok:
        raise RuntimeError(f"IK did not converge towards {target_pose}")
    return q


# --- geometria del tablero ---

FILES = "abcdefgh"

# configuracion enseñada en d5 (rad), la usamos para arrancar la IK en todos lados
D5_SEED_JOINTS = [d * np.pi / 180.0 for d in [59.03, -87.19, 111.60, -116.73, -88.17, 327.52]]

# (x, y, z) en metros, sacados de posiciones_reales.md
_D5_POS, _E4_POS, _GRAVEYARD_RANK5_POS = (
    (-0.05185, -0.32145, -0.35752), (0.01077, -0.38227, -0.36040), (-0.29999, -0.32143, -0.35758),
)
_D5_FILE, _D5_RANK, _E4_FILE, _E4_RANK, _GRAVEYARD_FILE = 4, 5, 5, 4, 0

# orientacion fija para todas las casillas (el mismo angulo de aproximacion
# para cualquier sitio del tablero), sacada de los puntos mas consistentes
ORIENTATION = (0.0425, -3.146, 0.081)
HOVER_HEIGHT = 0.0585  # medido comparando joints de hover y de agarre reales

REACHABLE_RANKS = range(2, 9)
GRAVEYARD_REACHABLE_SLOTS = range(4, 9)

# con estos 3 puntos enseñados (d5, e4 y graveyard rank5) interpolamos el
# resto del tablero: cuanto se desplaza la posicion por cada paso de fila/columna
FILE_AXIS = tuple((d - g) / (_D5_FILE - _GRAVEYARD_FILE) for d, g in zip(_D5_POS, _GRAVEYARD_RANK5_POS))
# e4-d5 es un paso en diagonal (+1 columna, -1 fila); quitamos la parte de columna
RANK_AXIS = tuple(
    (e - d - f * (_E4_FILE - _D5_FILE)) / (_E4_RANK - _D5_RANK)
    for e, d, f in zip(_E4_POS, _D5_POS, FILE_AXIS)
)


def _pos_for(file_idx, rank_idx):
    df, dr = file_idx - _D5_FILE, rank_idx - _D5_RANK
    return tuple(d + f * df + r * dr for d, f, r in zip(_D5_POS, FILE_AXIS, RANK_AXIS))


def square_pose(square):
    """Pose cartesiana de una casilla tipo "e4": (x, y, z, rx, ry, rz)."""
    return _pos_for(FILES.index(square[0].lower()) + 1, int(square[1:])) + ORIENTATION


def graveyard_pose(slot):
    """Pose cartesiana del slot 1-8 del graveyard (una columna a la izquierda de la fila a)."""
    return _pos_for(_GRAVEYARD_FILE, slot) + ORIENTATION


def hover_pose(pose):
    """La misma pose pero levantada HOVER_HEIGHT en Z."""
    x, y, z, rx, ry, rz = pose
    return (x, y, z + HOVER_HEIGHT, rx, ry, rz)


def joints_for(pose, seed=None):
    """Joints para llegar a una pose cartesiana, con seed opcional (si no, parte de d5)."""
    return inverse_kinematics(pose, seed if seed is not None else D5_SEED_JOINTS)


def _check_reachable(square):
    rank = int(square[1])
    if rank not in REACHABLE_RANKS:
        raise ValueError(f"square {square} (rank {rank}) is outside REACHABLE_RANKS {list(REACHABLE_RANKS)}")


class Location:
    """Una location del PDDL: una casilla o un slot del graveyard, con su hover."""

    def __init__(self, name, pose):
        self.name = name
        self.hover_name = f"{name}_hover"
        self.pose = pose
        self.hover_pose = hover_pose(pose)

    @classmethod
    def for_square(cls, square):
        _check_reachable(square)
        return cls(square, square_pose(square))

    @classmethod
    def for_graveyard(cls, slot):
        if slot not in GRAVEYARD_REACHABLE_SLOTS:
            raise ValueError(f"graveyard slot {slot} is outside {list(GRAVEYARD_REACHABLE_SLOTS)}")
        return cls(f"graveyard{slot}", graveyard_pose(slot))

    def pddl_facts(self):
        return [f"(is_hover {self.hover_name})", f"(above {self.hover_name} {self.name})", f"(valid_zone {self.name})"]


# --- generacion del XML para ktmpb ---

PI = 3.141592653589793
GRIPPER_CONTROL = 0.813  


def joints_to_controls(joints_rad):
    """6 angulos (rad) -> string de controles normalizados [0,1] + el de la pinza."""
    controls = []
    for i, q in enumerate(joints_rad):
        norm = (q + PI) / (2 * PI) if i == 2 else (q + 2 * PI) / (4 * PI)
        if not 0.0 <= norm <= 1.0:
            raise ValueError(f"joint {i} normalized to {norm:.4f}, outside [0,1] (q={q:.4f} rad)")
        controls.append(norm)
    controls.append(GRIPPER_CONTROL)
    return " ".join(f"{v:.6f}" for v in controls)


# home enseñado directamente, bien por encima del tablero (ver "home" en posiciones_reales.md)
HOME_JOINTS_DEG = [79.77, -80.75, 55.31, -68.87, -87.87, 348.59]
HOME_JOINTS = [d * np.pi / 180.0 for d in HOME_JOINTS_DEG]
HOME_CONTROLS = joints_to_controls(HOME_JOINTS)


def _move_xml(region_from, region_to, init_controls, goal_controls):
    return (
        f'<Move robot="UR3A" region_from="{region_from}" region_to="{region_to}">\n'
        f"    <Rob> ur3_right </Rob>\n    <Cont>controls/right_ur3_with_gripper.cntr</Cont>\n"
        f"    <InitControls> {init_controls} </InitControls>\n"
        f"    <GoalControls> {goal_controls} </GoalControls>\n</Move>"
    )


def tampconfig_move_actions(loc, seed=None):
    """Los 4 <Move> de una location: home<->hover y hover<->casilla, en los dos sentidos."""
    hover_j = joints_for(loc.hover_pose, seed=seed)
    square_j = joints_for(loc.pose, seed=hover_j)
    hover_c, square_c = joints_to_controls(hover_j), joints_to_controls(square_j)
    region = loc.name.upper()
    snippets = [
        _move_xml("HOME", f"{region}_HOVER", HOME_CONTROLS, hover_c),
        _move_xml(f"{region}_HOVER", region, hover_c, square_c),
        _move_xml(region, f"{region}_HOVER", square_c, hover_c),
        _move_xml(f"{region}_HOVER", "HOME", hover_c, HOME_CONTROLS),
    ]
    return snippets, square_j, hover_j


def tampconfig_hover_transfer(loc_a, loc_b, hover_a, hover_b):
    """<Move> directo entre los hover de dos locations, para cuando se lleva una pieza sin pasar por home."""
    region_a, region_b = f"{loc_a.name.upper()}_HOVER", f"{loc_b.name.upper()}_HOVER"
    return _move_xml(region_a, region_b, joints_to_controls(hover_a), joints_to_controls(hover_b))


def tampconfig_pick_or_place(tag, piece, kautham_name, loc, square_joints, hover_joints):
    # HomeControls es donde PICK/PLACE se retiran nada mas agarrar/soltar - se
    # pone el hover de esta location (no el home de verdad) para que suba en
    # vertical primero, en vez de cruzar el tablero a la altura de agarre y
    # arriesgarse a tirar otra pieza
    grasp_controls = joints_to_controls(square_joints)
    hover_controls = joints_to_controls(hover_joints)
    extra = "\n    <Link> robotiq_85_base_link </Link>" if tag == "Pick" else ""
    return (
        f'<{tag} robot="UR3A" object="{piece.upper()}" region="{loc.name.upper()}">\n'
        f"    <Rob> ur3_right </Rob>\n    <Obj> {kautham_name} </Obj>{extra}\n"
        f"    <Cont>controls/right_ur3_with_gripper.cntr</Cont>\n"
        f"    <HomeControls> {hover_controls} </HomeControls>\n"
        f'    <GraspControls grasp="topgrasp"> {grasp_controls} </GraspControls>\n</{tag}>'
    )
