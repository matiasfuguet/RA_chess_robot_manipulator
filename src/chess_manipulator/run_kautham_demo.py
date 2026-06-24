"""Run the original e4xd5 demo using the Kautham-calibrated kinematics."""

import os
import sys
import xml.etree.ElementTree as ET

import numpy as np

FILE_DIR = os.path.dirname(os.path.abspath(__file__))
KTMPB_CLIENT_DIR = os.path.expanduser(
    "~/ws_tamp/src/task_and_motion_planning2/ktmpb/ktmpb_client/ktmpb_client"
)
for _path in (FILE_DIR, KTMPB_CLIENT_DIR):
    if _path not in sys.path:
        sys.path.append(_path)

import kautham_square_to_joints as kj
from taskfile_simplify import simplify_taskfile

PI = np.pi
GRIPPER_CONTROL = 0.813
HOME_CONTROLS_LIST = [0.500, 0.375, 0.500, 0.375, 0.500, 0.500, GRIPPER_CONTROL]
HOVER_HEIGHT = 0.0585  # matches the real robot's hover height

# Graveyard pose validated in Kautham like the d5/e4 configs.
GRAVEYARD_GRASP_CONTROLS = [0.400, 0.390, 0.840, 0.320, 0.375, 0.375]


def joints_to_controls(joints_rad):
    controls = [(q + PI) / (2 * PI) if i == 2 else (q + 2 * PI) / (4 * PI) for i, q in enumerate(joints_rad)]
    return " ".join(f"{v:.6f}" for v in controls + [GRIPPER_CONTROL])


HOME_CONTROLS = " ".join(f"{v:.6f}" for v in HOME_CONTROLS_LIST)
HOME_JOINTS = kj.controls_to_joints(HOME_CONTROLS_LIST[:6])


def _move_xml(region_from, region_to, init_c, goal_c):
    return (
        f'<Move robot="UR3A" region_from="{region_from}" region_to="{region_to}">\n'
        f"    <Rob> ur3_right </Rob>\n    <Cont>controls/right_ur3_with_gripper.cntr</Cont>\n"
        f"    <InitControls> {init_c} </InitControls>\n    <GoalControls> {goal_c} </GoalControls>\n</Move>"
    )


def _pick_or_place_xml(tag, piece, kautham_name, region, grasp_c, hover_c):
    """Build a Pick/Place snippet that retreats to this location's hover."""
    extra = "\n    <Link> robotiq_85_base_link </Link>" if tag == "Pick" else ""
    return (
        f'<{tag} robot="UR3A" object="{piece}" region="{region}">\n'
        f"    <Rob> ur3_right </Rob>\n    <Obj> {kautham_name} </Obj>{extra}\n"
        f"    <Cont>controls/right_ur3_with_gripper.cntr</Cont>\n"
        f"    <HomeControls> {hover_c} </HomeControls>\n"
        f'    <GraspControls grasp="topgrasp"> {grasp_c} </GraspControls>\n</{tag}>'
    )


def build_location(name, grasp_joints):
    """Build movement snippets and hover joints for one location."""
    grasp_pose = kj.forward_kinematics(grasp_joints)
    x, y, z, rx, ry, rz = grasp_pose
    hover_pose = (x, y, z + HOVER_HEIGHT, rx, ry, rz)
    hover_joints = kj.inverse_kinematics(hover_pose, grasp_joints)

    grasp_c, hover_c = joints_to_controls(grasp_joints), joints_to_controls(hover_joints)
    region = name.upper()
    moves = [
        _move_xml("HOME", f"{region}_HOVER", HOME_CONTROLS, hover_c),
        _move_xml(f"{region}_HOVER", region, hover_c, grasp_c),
        _move_xml(region, f"{region}_HOVER", grasp_c, hover_c),
        _move_xml(f"{region}_HOVER", "HOME", hover_c, HOME_CONTROLS),
    ]
    return moves, grasp_c, hover_c, hover_joints


def build_actions_list():
    import MOVE
    import PICK
    import PLACE

    d5_joints = kj.controls_to_joints([0.581986, 0.378903, 0.810000, 0.337875, 0.377542, 0.954889])
    e4_joints = kj.controls_to_joints([0.599500, 0.395347, 0.775639, 0.336583, 0.380431, 0.972694])
    grave_joints = kj.controls_to_joints(GRAVEYARD_GRASP_CONTROLS)

    actions_list = []
    grasp_controls = {}
    hover_controls = {}
    keep_joints = [d5_joints, e4_joints, grave_joints]
    for name, joints in [("d5", d5_joints), ("e4", e4_joints), ("graveyard", grave_joints)]:
        moves, grasp_c, hover_c, hover_j = build_location(name, joints)
        grasp_controls[name] = grasp_c
        hover_controls[name] = hover_c
        keep_joints.append(hover_j)
        for snippet in moves:
            elem = ET.fromstring(snippet)
            actions_list.append({"tag": elem.tag, "attrib": dict(elem.attrib), "data": MOVE.Move_read(elem)})

    # Allow direct hover-to-hover transfers while carrying a piece.
    names = list(hover_controls.keys())
    for i, name_a in enumerate(names):
        for name_b in names[i + 1:]:
            snippet = _move_xml(f"{name_a.upper()}_HOVER", f"{name_b.upper()}_HOVER", hover_controls[name_a], hover_controls[name_b])
            elem = ET.fromstring(snippet)
            actions_list.append({"tag": elem.tag, "attrib": dict(elem.attrib), "data": MOVE.Move_read(elem)})

    for tag, piece, kautham_name, region in [
        ("Pick", "peon_negro", "PEON_NEGRO", "d5"),
        ("Place", "peon_negro", "PEON_NEGRO", "graveyard"),
        ("Pick", "peon_blanco", "PEON_BLANCO", "e4"),
        ("Place", "peon_blanco", "PEON_BLANCO", "d5"),
    ]:
        snippet = _pick_or_place_xml(tag, piece, kautham_name, region.upper(), grasp_controls[region], hover_controls[region])
        elem = ET.fromstring(snippet)
        reader = PICK.Pick_read if tag == "Pick" else PLACE.Place_read
        actions_list.append({"tag": elem.tag, "attrib": dict(elem.attrib), "data": reader(elem)})

    return actions_list, keep_joints


def _drop_redundant_hover_moves(plan_lines):
    """Drop square->hover moves already done inside PICK/PLACE."""
    result = []
    for line in plan_lines:
        parts = line.split()
        if parts[0].upper() == "MOVE" and parts[3].upper() == f"{parts[2].upper()}_HOVER" and result:
            prev_parts = result[-1].split()
            if prev_parts[0].upper() in ("PICK", "PLACE") and prev_parts[3].upper() == parts[2].upper():
                continue
        result.append(line)
    return result


def get_combined_plan():
    import rclpy
    import ktmpb_python_interface as ktmpb

    with open(os.path.join(FILE_DIR, "ff-domains", "domain_chess.pddl")) as f:
        domain_text = f.read()
    with open(os.path.join(FILE_DIR, "ff-domains", "problem_chess.pddl")) as f:
        problem_text = f.read()

    rclpy.init()
    client = ktmpb.DownwardClient()
    result = client.send_request(problem_text, domain_text, "", "")
    plan = [line for line in result.plan if line.strip() and not line.startswith(";")]
    client.destroy_node()
    rclpy.shutdown()
    return plan


def run(models_folder_path, scenario_folder_path, show_rviz=False, include_objects=True):
    """Run the demo and write its taskfile."""
    import rclpy
    from rclpy.node import Node
    import kautham_ros.kautham_ros_interface_python as kautham
    import ktmpb_python_interface as ktmpb
    import MOVE  # noqa: F401
    import PICK  # noqa: F401
    import PLACE  # noqa: F401

    combined_plan = _drop_redundant_hover_moves(get_combined_plan())
    if not include_objects:
        combined_plan = [line for line in combined_plan if line.strip().lower().startswith("move")]
    print(f"Plan ({len(combined_plan)} lines):")
    for line in combined_plan:
        print(" ", line)

    actions_list, keep_joints = build_actions_list()
    keep_joints.append(HOME_JOINTS)

    object_world_poses = {
        "PEON_NEGRO": list(kj.PEON_NEGRO_WORLD_POSE),
        "PEON_BLANCO": list(kj.PEON_BLANCO_WORLD_POSE),
    } if include_objects else {}

    scene_file = "OMPL_RRTConnect_chess_pawn_capture.xml" if include_objects else "OMPL_RRTConnect_chess_pawn_capture_no_objects.xml"

    rclpy.init()
    node = Node("kautham_demo_runner")
    node.show_rviz = show_rviz

    kautham.kOpenProblem(node, models_folder_path, os.path.join(scenario_folder_path, scene_file))
    for name, pose in object_world_poses.items():
        kautham.kSetObstaclePos(node, name, pose)

    kautham.kSetRobControlsNoQuery(node, "controls/right_ur3_with_gripper.cntr")
    kautham.kSetQuery(node, HOME_CONTROLS_LIST, [])
    ktmpb.ktmpbMoveRobot(node, controls=HOME_CONTROLS_LIST, sample_type="init")

    taskfile_name = "taskfile_kautham_demo.xml" if include_objects else "taskfile_kautham_demo_no_objects.xml"
    taskfile_path = os.path.join(scenario_folder_path, taskfile_name)
    info = ktmpb.knowledge()
    info.taskfile = open(taskfile_path, "w+")
    info.taskfile.write(f'<?xml version="1.0"?>\n<Task name="{scene_file}">\n\t<Initialstate>\n')
    for name, pose in object_world_poses.items():
        info.taskfile.write(f'\t\t<Object object="{name}"> {" ".join(str(x) for x in pose)} </Object>\n')
    info.taskfile.write("\t</Initialstate>\n")

    for line in combined_plan:
        action = ktmpb.find_action_for_plan_line(line, actions_list)
        if not action:
            node.get_logger().error(f"Could not match action for line: {line}")
            continue
        parts = line.split()
        func = getattr(sys.modules[parts[0].upper()], parts[0].upper())
        if func(node, action["data"], info, parts) is False:
            node.get_logger().error(f"Action {parts[0]} failed for line: {line}")
            break

    info.taskfile.write("</Task>\n")
    info.taskfile.close()
    simplify_taskfile(taskfile_path, keep_joints=keep_joints)
    node.get_logger().info(f"Results saved in {taskfile_path}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    # models_folder_path="" on purpose: the scene file's model paths are absolute,
    # and kOpenProblem prepends this folder, so a non-empty one would double it.
    run(models_folder_path="", scenario_folder_path=FILE_DIR, include_objects="--no-objects" not in sys.argv)
