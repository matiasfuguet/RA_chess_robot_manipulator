"""Given a board square (or graveyard slot), get a UR3e joint config and
the matching ktmpb tampconfig Action XML. Three parts, in one file because
they're really one job:

1. Kinematics: forward/inverse kinematics for the UR3e arm, chain taken
   verbatim from Kautham's own UR3e description
   (/usr/share/kautham/demos/models/robots/robot_descriptions/UR/config/ur3e/default_kinematics.yaml),
   with a calibrated BASE_OFFSET correction (the bare-arm chain's output
   was off from the real robot's recorded frame by a near-constant ~348mm
   in Z - most likely an unmodeled stand/riser - fit from 3 known points).

2. Board geometry: interpolates Cartesian poses for any square/graveyard
   slot from those same 3 calibration points (d5, e4, graveyard rank 5 -
   see posiciones_reales.md).

3. tampconfig generation: turns a Location + joint config into the
   <Move>/<Pick>/<Place> XML snippets ktmpb's tampconfig format expects.

Known limitation: rank 1, the rank-2 corners (a2/h2), and graveyard slots
2-3's hover pose don't converge with this solver - either a genuine reach
limit that far from where d5/e4 were taught, or a branch the locally
seeded search can't find. REACHABLE_RANKS/GRAVEYARD_REACHABLE_SLOTS scope
around this for now.
"""

import numpy as np

# ---------------------------------------------------------------- kinematics

# (xyz, rpy) per segment, base -> shoulder_pan -> ... -> wrist_3.
SEGMENTS = [
    ((0, 0, 0.15185), (0, 0, 0)),
    ((0, 0, 0), (1.570796327, 0, 0)),
    ((-0.24355, 0, 0), (0, 0, 0)),
    ((-0.2132, 0, 0.13105), (0, 0, 0)),
    ((0, -0.08535, -1.750557762378351e-11), (1.570796327, 0, 0)),
    ((0, 0.0921, -1.8890025766262e-11), (1.570796326589793, 3.141592653589793, 3.141592653589793)),
]
# TCP offset (gripper fingertip) from abrir_pinza.py/cerrar_pinza.py's set_tcp(p[0,0,0.2286,0,0,0]).
TCP_OFFSET = np.array([0.0, 0.0, 0.2286])
BASE_OFFSET = np.array([-0.015467, 0.013733, -0.347767])  # see module docstring


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
    """6 joint angles (rad) -> (x, y, z, rx, ry, rz) TCP pose in robot frame."""
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


def inverse_kinematics(target_pose, seed_joints, max_iters=200, tol=1e-8, num_steps=10):
    """Damped least-squares IK, seeded from a nearby known joint config so
    it converges to the same kinematic branch the real robot was taught
    with. Falls back to walking through num_steps interpolated waypoints
    if the seed is too far for direct convergence (a big board jump) -
    interpolating orientation via the *relative* rotation, since two
    rotvecs near theta=pi can look like near-opposites component-wise
    while representing nearly the same rotation (axis-angle sign
    ambiguity), which would otherwise make naive lerp build a nonsense
    intermediate target."""
    q, ok = _ik_attempt(target_pose, seed_joints, max_iters, tol)
    if ok:
        return q

    start_pose = forward_kinematics(seed_joints)
    start_pos, start_rot = start_pose[:3], _rotvec_to_rotmat(start_pose[3:])
    target_pos, target_rot = np.array(target_pose[:3]), _rotvec_to_rotmat(np.array(target_pose[3:]))
    relative_rotvec = _rotmat_to_rotvec(target_rot @ start_rot.T)

    q = np.array(seed_joints, dtype=float)
    for step in range(1, num_steps + 1):
        alpha = step / num_steps
        waypoint_rot = _rotvec_to_rotmat(alpha * relative_rotvec) @ start_rot
        waypoint = np.concatenate([start_pos + alpha * (target_pos - start_pos), _rotmat_to_rotvec(waypoint_rot)])
        q, ok = _ik_attempt(waypoint, q, max_iters, tol)
        if not ok:
            raise RuntimeError(f"IK did not converge at waypoint {step}/{num_steps} towards {target_pose}")
    return q


# ------------------------------------------------------------- board geometry

FILES = "abcdefgh"

# d5's known joint config (rad) - central reference, used to seed IK for
# any other square so the solver converges to the real robot's branch.
D5_SEED_JOINTS = [d * np.pi / 180.0 for d in [59.03, -87.19, 111.60, -116.73, -88.17, 327.52]]

# (x, y, z) in metres, from posiciones_reales.md (mm converted to m).
_D5_POS, _E4_POS, _GRAVEYARD_RANK5_POS = (
    (-0.05185, -0.32145, -0.35752), (0.01077, -0.38227, -0.36040), (-0.29999, -0.32143, -0.35758),
)
_D5_FILE, _D5_RANK, _E4_FILE, _E4_RANK, _GRAVEYARD_FILE = 4, 5, 5, 4, 0

# d5 and the graveyard agree on orientation; e4's recorded value is the
# noisy outlier (taught by hand, ~2-3 degrees off) so it's not used here.
ORIENTATION = (0.0425, -3.146, 0.081)
# Hover is a pure +Z translation, calibrated from real hover-vs-grasp
# joint pairs (~58.5mm, not the 10cm originally envisioned).
HOVER_HEIGHT = 0.0585

REACHABLE_RANKS = range(2, 9)
GRAVEYARD_REACHABLE_SLOTS = range(4, 9)

# file_axis: displacement per +1 file step, from d5 -> graveyard (same
# rank, so purely along the file direction).
FILE_AXIS = tuple((d - g) / (_D5_FILE - _GRAVEYARD_FILE) for d, g in zip(_D5_POS, _GRAVEYARD_RANK5_POS))
# rank_axis: e4 - d5 is one diagonal step (+1 file, -1 rank); subtract the
# file component to isolate the rank component.
RANK_AXIS = tuple(
    (e - d - f * (_E4_FILE - _D5_FILE)) / (_E4_RANK - _D5_RANK)
    for e, d, f in zip(_E4_POS, _D5_POS, FILE_AXIS)
)


def _pos_for(file_idx, rank_idx):
    df, dr = file_idx - _D5_FILE, rank_idx - _D5_RANK
    return tuple(d + f * df + r * dr for d, f, r in zip(_D5_POS, FILE_AXIS, RANK_AXIS))


def square_pose(square):
    """Cartesian pose for a board square like "e4": (x, y, z, rx, ry, rz)."""
    return _pos_for(FILES.index(square[0].lower()) + 1, int(square[1:])) + ORIENTATION


def graveyard_pose(slot):
    """Cartesian pose for graveyard slot 1-8 (one column left of file a)."""
    return _pos_for(_GRAVEYARD_FILE, slot) + ORIENTATION


def hover_pose(pose):
    """Pre-grasp approach pose: same pose, lifted HOVER_HEIGHT in Z."""
    x, y, z, rx, ry, rz = pose
    return (x, y, z + HOVER_HEIGHT, rx, ry, rz)


def joints_for(pose, seed=None):
    """UR3e joint config (rad) for a Cartesian pose, seeded from d5's
    known config or a caller-supplied seed (e.g. the previous location's
    solution, for smoother continuity along a planned path)."""
    return inverse_kinematics(pose, seed if seed is not None else D5_SEED_JOINTS)


def _check_reachable(square):
    rank = int(square[1])
    if rank not in REACHABLE_RANKS:
        raise ValueError(f"square {square} (rank {rank}) is outside REACHABLE_RANKS {list(REACHABLE_RANKS)}")


class Location:
    """One PDDL location: a square or graveyard slot, plus its hover."""

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


# --------------------------------------------------------- tampconfig XML

PI = 3.141592653589793
GRIPPER_CONTROL = 0.813  # constant placeholder; actual open/close is done via
# kAttachObject/kDetachObject in PICK.py/PLACE.py, not this control vector.


def joints_to_controls(joints_rad):
    """6 joint angles (rad) -> normalized [0,1] control vector string (+
    constant gripper slot), using the corrected formula from posiciones_reales.md."""
    controls = []
    for i, q in enumerate(joints_rad):
        norm = (q + PI) / (2 * PI) if i == 2 else (q + 2 * PI) / (4 * PI)  # joint 3 (elbow) differs
        if not 0.0 <= norm <= 1.0:
            raise ValueError(f"joint {i} normalized to {norm:.4f}, outside [0,1] (q={q:.4f} rad)")
        controls.append(norm)
    controls.append(GRIPPER_CONTROL)
    return " ".join(f"{v:.6f}" for v in controls)


HOME_CONTROLS = joints_to_controls(D5_SEED_JOINTS)


def _move_xml(region_from, region_to, init_controls, goal_controls):
    return (
        f'<Move robot="UR3A" region_from="{region_from}" region_to="{region_to}">\n'
        f"    <Rob> ur3_right </Rob>\n    <Cont>controls/right_ur3_with_gripper.cntr</Cont>\n"
        f"    <InitControls> {init_controls} </InitControls>\n"
        f"    <GoalControls> {goal_controls} </GoalControls>\n</Move>"
    )


def tampconfig_move_actions(loc, seed=None):
    """3 <Move> XML snippets: HOME<->hover, hover<->square. Returns
    (snippets, square_joints) - square_joints is reused by the matching
    Pick/Place GraspControls."""
    hover_j = joints_for(loc.hover_pose, seed=seed)
    square_j = joints_for(loc.pose, seed=hover_j)
    hover_c, square_c = joints_to_controls(hover_j), joints_to_controls(square_j)
    region = loc.name.upper()
    snippets = [
        _move_xml("HOME", f"{region}_HOVER", HOME_CONTROLS, hover_c),
        _move_xml(f"{region}_HOVER", region, hover_c, square_c),
        _move_xml(region, "HOME", square_c, HOME_CONTROLS),
    ]
    return snippets, square_j


def tampconfig_pick_or_place(tag, piece, kautham_name, loc, square_joints):
    """piece is the logical PDDL/plan-line identifier (e.g. 'e4_piece') -
    used for the object= attribute that find_action_for_plan_line matches
    against. kautham_name is the actual Kautham scene object this piece
    currently represents (e.g. 'PEON_BLANCO') - used in <Obj>, which
    PICK.py/PLACE.py pass straight to kAttachObject/kDetachObject."""
    grasp_controls = joints_to_controls(square_joints)
    extra = "\n    <Link> robotiq_85_base_link </Link>" if tag == "Pick" else ""
    return (
        f'<{tag} robot="UR3A" object="{piece.upper()}" region="{loc.name.upper()}">\n'
        f"    <Rob> ur3_right </Rob>\n    <Obj> {kautham_name} </Obj>{extra}\n"
        f"    <Cont>controls/right_ur3_with_gripper.cntr</Cont>\n"
        f"    <HomeControls> {HOME_CONTROLS} </HomeControls>\n"
        f'    <GraspControls grasp="topgrasp"> {grasp_controls} </GraspControls>\n</{tag}>'
    )


if __name__ == "__main__":
    # Cross-check against the 3 known real-robot points.
    known = {
        "d5": ([59.03, -87.19, 111.60, -116.73, -88.17, 327.52], (-0.05185, -0.32145, -0.35752, 0.042, -3.146, 0.081)),
        "e4": ([71.64, -75.35, 99.23, -117.66, -86.09, 340.34], (0.01077, -0.38227, -0.36040, 0.040, -3.182, 0.134)),
        "graveyard5": ([30.45, -59.38, 73.70, -105.48, -87.28, 292.92], (-0.29999, -0.32143, -0.35758, 0.043, -3.146, 0.081)),
    }
    for name, (joints_deg, measured) in known.items():
        computed = forward_kinematics([d * np.pi / 180 for d in joints_deg])
        err_mm = np.linalg.norm(computed[:3] - np.array(measured[:3])) * 1000
        print(f"FK {name}: err={err_mm:.2f}mm")

    loc = Location.for_square("e4")
    snippets, square_j = tampconfig_move_actions(loc, seed=D5_SEED_JOINTS)
    print()
    print(f"=== tampconfig snippets for {loc.name} ===")
    for s in snippets:
        print(s)
    print(tampconfig_pick_or_place("Pick", "e4_piece", "PEON_BLANCO", loc, square_j))
