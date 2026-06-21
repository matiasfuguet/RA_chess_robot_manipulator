"""Kautham-visualization-only counterpart to square_to_joints.py.

square_to_joints.py is calibrated purely against the REAL robot's measured
data and must stay that way - it's what eventually drives the real UR3e.
This module is separate on purpose: Kautham's scene uses a *different*
robot model (the scene's xacro defaults ur_type to "ur3", not "ur3e" -
see ur3_robotiq_85_gripper.urdf.xacro) and a *different* gripper
(robotiq_85 in simulation vs. the real OnRobot RG2), so the same joint
values that correctly grasp a piece on the real robot land ~17cm off in
Kautham. This is exactly the same kind of mismatch CHANGES.md already
documented; this module fixes it for visualization without touching
anything the real robot depends on.

Calibrated against tampconfig_chess.xml's existing validated d5/e4 grasp
joint configs (the SAME joint angles used in tampconfig_chess_real.xml,
per CHANGES.md - "the <Actions> block is identical in both files") and
their corresponding world-frame piece positions already defined in
OMPL_RRTConnect_chess_pawn_capture.xml.

Found while calibrating: the robot's world-placement rotation must use
the *negative* of the XML's stated TH (matches CHANGES.md's prose -
"rotated -pi/2 around Z" - even though the raw <Home TH=.../> value is
positive; Kautham's axis-angle sign convention here is the opposite of
the naive reading). With that fix, X/Y matched the known piece positions
to sub-millimeter precision; only a consistent ~169mm Z residual
remained, absorbed into BASE_OFFSET below (almost certainly the
robotiq_85 gripper's TCP length, which differs from the real gripper's).
"""

import numpy as np

# UR3 (NOT ur3e) kinematics - /usr/share/kautham/demos/models/robots/robot_descriptions/UR/config/ur3/default_kinematics.yaml
SEGMENTS = [
    ((0, 0, 0.1519), (0, 0, 0)),
    ((0, 0, 0), (1.570796327, 0, 0)),
    ((-0.24365, 0, 0), (0, 0, 0)),
    ((-0.21325, 0, 0.11235), (0, 0, 0)),
    ((0, -0.08535, -1.750557762378351e-11), (1.570796327, 0, 0)),
    ((0, 0.0819, -1.679797079540562e-11), (1.570796326589793, 3.141592653589793, 3.141592653589793)),
]
BASE_OFFSET = np.array([0.000255, -0.000243, -0.168594])  # see module docstring


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


def inverse_kinematics(target_pose, seed_joints, max_iters=200, tol=1e-8):
    """Damped least-squares IK for Kautham's UR3 model. See
    square_to_joints.inverse_kinematics for the same algorithm with the
    waypoint-stepping fallback, omitted here since this module is only
    used for the 2 known pieces so far, both within easy reach of d5/e4."""
    q, ok = _ik_attempt(target_pose, seed_joints, max_iters, tol)
    if not ok:
        raise RuntimeError(f"IK did not converge towards {target_pose}")
    return q


def _axis_angle_to_matrix(x, y, z, wx, wy, wz, th):
    axis = np.array([wx, wy, wz])
    n = np.linalg.norm(axis)
    r = _rotvec_to_rotmat(axis / n * th if n > 1e-9 else np.zeros(3))
    t = np.eye(4)
    t[:3, :3] = r
    t[:3, 3] = [x, y, z]
    return t


# Robot placement in Kautham world frame, from OMPL_RRTConnect_chess_pawn_capture.xml's
# <Robot><Home TH="1.570796327" WZ="1.0" WY="0.0" WX="0.0" Z="0.0" Y="0.0" X="0.37" />.
# Note the negated TH - see module docstring.
_WORLD_T_ROBOT = _axis_angle_to_matrix(0.37, 0, 0, 0, 0, 1, -1.570796327)
_ROBOT_T_WORLD = np.linalg.inv(_WORLD_T_ROBOT)


def world_pose_to_robot_frame(x, y, z, wx, wy, wz, th):
    """Converts a Kautham world-frame object pose (Kautham's own axis-angle
    attribute order TH,WZ,WY,WX,Z,Y,X, but passed here as x,y,z,wx,wy,wz,th
    for clarity) into (x,y,z,rx,ry,rz) in the robot's own base frame, ready
    for inverse_kinematics()."""
    world_t_obj = _axis_angle_to_matrix(x, y, z, wx, wy, wz, th)
    robot_t_obj = _ROBOT_T_WORLD @ world_t_obj
    return tuple(robot_t_obj[:3, 3]) + tuple(_rotmat_to_rotvec(robot_t_obj[:3, :3]))


PI = np.pi


def controls_to_joints(controls):
    """Inverse of square_to_joints.joints_to_controls's normalization."""
    return [c * 2 * PI - PI if i == 2 else c * 4 * PI - 2 * PI for i, c in enumerate(controls)]


# Known-good simulation grasp configs, from tampconfig_chess.xml's existing
# Pick D5 / Pick E4 GraspControls (already correct for Kautham, used here
# only to calibrate/validate BASE_OFFSET above - not needed at runtime).
_D5_GRASP_CONTROLS = [0.581986, 0.378903, 0.810000, 0.337875, 0.377542, 0.954889]
_E4_GRASP_CONTROLS = [0.599500, 0.395347, 0.775639, 0.336583, 0.380431, 0.972694]

# Known world-frame piece poses, from OMPL_RRTConnect_chess_pawn_capture.xml.
PEON_NEGRO_WORLD_POSE = (0.058, 0.053, 0.060, 0.013345, -0.999580, 0.025736, 3.147323)
PEON_BLANCO_WORLD_POSE = (0.003, -0.003, 0.057, 0.012559, -0.999036, 0.042071, 3.185071)


if __name__ == "__main__":
    for name, controls, world_pose in [
        ("d5/PEON_NEGRO", _D5_GRASP_CONTROLS, PEON_NEGRO_WORLD_POSE),
        ("e4/PEON_BLANCO", _E4_GRASP_CONTROLS, PEON_BLANCO_WORLD_POSE),
    ]:
        joints = controls_to_joints(controls)
        computed = forward_kinematics(joints)
        target = world_pose_to_robot_frame(*world_pose)
        err_mm = np.linalg.norm(computed[:3] - np.array(target[:3])) * 1000
        print(f"{name}: computed_pos={np.round(computed[:3], 4)} target_pos={np.round(target[:3], 4)} err={err_mm:.2f}mm")
