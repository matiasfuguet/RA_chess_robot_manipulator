"""Run a list of UCI moves through the TAMP pipeline: solve each move's PDDL
problem in game order (the manipulation domain knows nothing about chess
legality, so the moves can't be one big problem), concatenate the plans, then
replay the whole sequence through ktmpb's MOVE/PICK/PLACE loop to generate a
Kautham taskfile.

Needs the downward_server and the kautham_ros node running (see README).
"""

import os
import sys
import xml.etree.ElementTree as ET

FILE_DIR = os.path.dirname(os.path.abspath(__file__))
# ktmpb_client bare-imports its own modules (MOVE/PICK/PLACE/ktmpb_python_interface)
# by appending its directory to sys.path, so we import them the same way.
KTMPB_CLIENT_DIR = os.path.expanduser(
    "~/ws_tamp/src/task_and_motion_planning2/ktmpb/ktmpb_client/ktmpb_client"
)
for _path in (FILE_DIR, KTMPB_CLIENT_DIR):
    if _path not in sys.path:
        sys.path.append(_path)

import square_to_joints as sj
from taskfile_simplify import simplify_taskfile

DOMAIN_NAME = "chesscapture"


def parse_uci_move(move):
    move = move.strip().lower()
    if len(move) != 4:
        raise ValueError(f"expected UCI move like 'e2e4', got {move!r}")
    return move[:2], move[2:]


def next_free_graveyard_slot(used_slots):
    for slot in sj.GRAVEYARD_REACHABLE_SLOTS:
        if slot not in used_slots:
            return slot
    raise ValueError(f"all graveyard slots in {list(sj.GRAVEYARD_REACHABLE_SLOTS)} are occupied")


def _episode_pddl(piece, src_loc, dst_loc, extra_init_clear):
    """One pick-at-src / place-at-dst episode's :init/:goal fragments."""
    init = [f"(in {piece} {src_loc.name})"] + src_loc.pddl_facts() + dst_loc.pddl_facts()
    if extra_init_clear:
        init.append(f"(clear {dst_loc.name})")
    return init, f"(in {piece} {dst_loc.name})"


def build_move_episode(board, move_uci, used_graveyard_slots, problem_name="chess_move"):
    """Build one move's PDDL problem. board maps square->piece (state before the
    move). A capture becomes two episodes: captured piece -> graveyard, then
    capturing piece -> destination. Returns a dict with pddl_text, new_board,
    new_used_graveyard_slots, locations (for tampconfig generation) and pieces
    (the Pick/Place actions). Regular moves and captures only - no castling, en
    passant or promotion - and squares are limited to REACHABLE_RANKS."""
    src, dst = parse_uci_move(move_uci)
    if src not in board:
        raise ValueError(f"no piece on {src} to move")

    moving_piece = board[src]
    captured_piece = board.get(dst)
    src_loc, dst_loc = sj.Location.for_square(src), sj.Location.for_square(dst)

    init_facts = ["(at ur3a home)", "(handEmpty)", "(clear home)", "(is_home home)"]
    goal_facts = []
    locations = [src_loc, dst_loc]
    pieces = []
    new_board = dict(board)
    new_used = set(used_graveyard_slots)

    if captured_piece is not None:
        grave_loc = sj.Location.for_graveyard(next_free_graveyard_slot(used_graveyard_slots))
        locations.append(grave_loc)

        init, goal = _episode_pddl(captured_piece, dst_loc, grave_loc, extra_init_clear=True)
        init_facts += init
        goal_facts.append(goal)
        pieces += [("Pick", captured_piece, dst_loc), ("Place", captured_piece, grave_loc)]

        new_board.pop(dst)
        new_used.add(int(grave_loc.name.replace("graveyard", "")))
    else:
        init_facts.append(f"(clear {dst_loc.name})")  # dst empty, needed for place

    init, goal = _episode_pddl(moving_piece, src_loc, dst_loc, extra_init_clear=False)
    init_facts += init
    goal_facts.append(goal)
    pieces += [("Pick", moving_piece, src_loc), ("Place", moving_piece, dst_loc)]

    new_board[dst] = moving_piece
    new_board.pop(src)

    location_names = [name for loc in locations for name in (loc.name, loc.hover_name)] + ["home"]
    obstacles = [moving_piece] + ([captured_piece] if captured_piece else [])

    pddl_text = (
        f"(define (problem {problem_name})\n\n(:domain {DOMAIN_NAME})\n\n"
        f"(:objects\n    {' '.join(location_names)} - location\n"
        f"    {' '.join(obstacles)} - obstacle\n    ur3a - robot\n)\n\n"
        f"(:init\n    " + "\n    ".join(init_facts) + "\n)\n\n"
        f"(:goal\n    (and " + " ".join(goal_facts) + " (at ur3a home))\n)\n\n)\n"
    )

    return {
        "pddl_text": pddl_text,
        "new_board": new_board,
        "new_used_graveyard_slots": new_used,
        "locations": locations,
        "pieces": pieces,
    }


with open(os.path.join(FILE_DIR, "ff-domains", "domain_chess.pddl")) as _f:
    DOMAIN_TEXT = _f.read()


def build_combined_plan(board, moves, downward_client=None):
    """Solve every move and concatenate the plans. board maps square->piece
    (starting state); moves is a list of UCI strings. Returns (plan_lines,
    locations, pieces, final_board), locations/pieces deduplicated by name.

    Pass a downward_client to reuse an rclpy session; otherwise one is created
    and torn down here."""
    import rclpy
    import ktmpb_python_interface as ktmpb

    owns_session = downward_client is None
    if owns_session:
        rclpy.init()
        downward_client = ktmpb.DownwardClient()

    combined_plan = []
    locations = {}
    pieces = {}
    used_graveyard_slots = set()

    for move_uci in moves:
        episode = build_move_episode(board, move_uci, used_graveyard_slots)
        result = downward_client.send_request(episode["pddl_text"], DOMAIN_TEXT, "", "")
        combined_plan += [line for line in result.plan if line.strip() and not line.startswith(";")]

        for loc in episode["locations"]:
            locations[loc.name] = loc
        for action_type, piece, loc in episode["pieces"]:
            pieces[(action_type, piece, loc.name)] = (action_type, piece, loc)

        board = episode["new_board"]
        used_graveyard_slots = episode["new_used_graveyard_slots"]

    if owns_session:
        downward_client.destroy_node()
        rclpy.shutdown()

    return combined_plan, locations, pieces, board


def build_actions_list(locations, pieces, piece_to_kautham, seed=None):
    """Build the tampconfig Move/Pick/Place entries for every location/piece
    touched, parsed through ktmpb_client's own Move_read/Pick_read/Place_read so
    the dicts match what MOVE/PICK/PLACE expect. piece_to_kautham maps a logical
    piece name (e.g. 'e4_piece') to its Kautham scene object (e.g. 'PEON_BLANCO')."""
    import MOVE
    import PICK
    import PLACE

    # Seed each IK from the previous location's solution rather than a fixed
    # seed - chaining this way is what keeps IK converging across the board.
    seed = seed if seed is not None else sj.D5_SEED_JOINTS
    actions_list = []
    square_joints = {}

    for name, loc in locations.items():
        snippets, square_j = sj.tampconfig_move_actions(loc, seed=seed)
        square_joints[name] = square_j
        seed = square_j
        for snippet in snippets:
            elem = ET.fromstring(snippet)
            actions_list.append({"tag": elem.tag, "attrib": dict(elem.attrib), "data": MOVE.Move_read(elem)})

    for (action_type, piece, loc_name), (_, _, loc) in pieces.items():
        snippet = sj.tampconfig_pick_or_place(
            action_type, piece, piece_to_kautham[piece], loc, square_joints[loc_name]
        )
        elem = ET.fromstring(snippet)
        reader = PICK.Pick_read if action_type == "Pick" else PLACE.Place_read
        actions_list.append({"tag": elem.tag, "attrib": dict(elem.attrib), "data": reader(elem)})

    return actions_list


# World-frame poses (Kautham axis-angle) of the scene's two pieces. A game with
# other pieces needs matching <Obstacle> entries added to the scene XML first.
DEFAULT_OBJECT_WORLD_POSES = {
    "PEON_NEGRO": [0.058, 0.053, 0.060, 0.013345, -0.999580, 0.025736, 3.147323],
    "PEON_BLANCO": [0.003, -0.003, 0.057, 0.012559, -0.999036, 0.042071, 3.185071],
}
DEFAULT_PIECE_TO_KAUTHAM = {"e4_piece": "PEON_BLANCO", "e6_piece": "PEON_NEGRO"}
KAUTHAM_PROBLEM_FILE = "OMPL_RRTConnect_chess_pawn_capture.xml"
ROBOT_HOME_CONTROLS = [0.500, 0.375, 0.500, 0.375, 0.500, 0.500, 0.813]

# Where pieces sit (well out of reach) when parked in --no-objects mode. Each
# piece needs its own spot: attach keeps the object's world offset, so two
# pieces parked at the same point would drag into each other.
PARKING_SPOTS = {
    "PEON_NEGRO": [-0.8, 0.0, -1.0, 1.0, 0.0, 0.0, 0.0],
    "PEON_BLANCO": [0.8, 0.0, -1.0, 1.0, 0.0, 0.0, 0.0],
}

# Sideways (X) offset applied just before attaching: the real-robot grasp pose
# sits inside Kautham's simulated hand, and attach refuses an in-collision
# object. The closed hand has no vertical gap, but 8cm sideways clears it.
ATTACH_CLEARANCE_X = 0.08

# Raise Kautham's planning budget from the scene default (10s) - RRTConnect is
# randomized and was timing out before finding the narrow gap, making the same
# setup pass on some runs and fail on others.
MAX_PLANNING_TIME_SECONDS = "60"


def _drop_redundant_home_moves(plan_lines):
    """PICK/PLACE already retreat to HOME inside their own Kautham execution.
    The plan still emits a separate symbolic 'move rob region home' after each
    (the planner needs it for state-tracking), but re-solving that transition
    with the object still attached routes through the constrained planner and
    throws. Drop those lines."""
    result = []
    for line in plan_lines:
        parts = line.split()
        if parts[0].upper() == "MOVE" and parts[3].upper() == "HOME" and result:
            prev_parts = result[-1].split()
            if prev_parts[0].upper() in ("PICK", "PLACE") and prev_parts[3].upper() == parts[2].upper():
                continue
        result.append(line)
    return result


def run_on_kautham(combined_plan, locations, pieces, models_folder_path,
                    scenario_folder_path, piece_to_kautham=None,
                    object_world_poses=None, show_rviz=False, include_objects=True):
    """Replay the combined plan through Kautham and write the taskfile. Needs the
    kautham_ros node already running.

    With include_objects=False the pieces stay in the scene (Pick/Place look them
    up by name and Kautham segfaults if they're missing) but get parked far away
    while not held, so the real-robot grasp pose - which doesn't match Kautham's
    simulated UR3+robotiq_85 - can't be rejected for grazing a piece mesh.
    kAttachObject/kDetachObject are monkeypatched (on the same cached module
    PICK/PLACE import) to restore a piece to its last-known pose - offset by
    ATTACH_CLEARANCE_X - right before attaching, and re-park it after detaching."""
    import rclpy
    from rclpy.node import Node
    import kautham_ros.kautham_ros_interface_python as kautham
    import ktmpb_python_interface as ktmpb
    import MOVE  # noqa: F401  (needed in globals() for ktmpb's dispatch-by-tag)
    import PICK  # noqa: F401
    import PLACE  # noqa: F401

    piece_to_kautham = piece_to_kautham or DEFAULT_PIECE_TO_KAUTHAM
    real_object_poses = object_world_poses or DEFAULT_OBJECT_WORLD_POSES
    if include_objects:
        object_world_poses = real_object_poses
    else:
        last_known_pose = dict(real_object_poses)
        object_world_poses = {name: PARKING_SPOTS[name] for name in real_object_poses}
        real_attach, real_detach = kautham.kAttachObject, kautham.kDetachObject

        def _attach_at_last_known_pose(node_, robot_name, link_name, obsname):
            lifted = list(last_known_pose[obsname])
            lifted[0] += ATTACH_CLEARANCE_X
            kautham.kSetObstaclePos(node_, obsname, lifted)
            return real_attach(node_, robot_name, link_name, obsname)

        def _detach_and_park(node_, obsname):
            result = real_detach(node_, obsname)
            pose = kautham.kGetObstaclePos(node_, obsname)
            if pose:
                last_known_pose[obsname] = list(pose)
            kautham.kSetObstaclePos(node_, obsname, PARKING_SPOTS[obsname])
            return result

        kautham.kAttachObject = _attach_at_last_known_pose
        kautham.kDetachObject = _detach_and_park
    actions_list = build_actions_list(locations, pieces, piece_to_kautham)

    rclpy.init()
    node = Node("chess_game_runner")
    node.show_rviz = show_rviz

    kautham.kOpenProblem(node, models_folder_path, os.path.join(scenario_folder_path, KAUTHAM_PROBLEM_FILE))
    kautham.kSetPlannerParameter(node, "_Max Planning Time", MAX_PLANNING_TIME_SECONDS)
    for kautham_name, pose in object_world_poses.items():
        kautham.kSetObstaclePos(node, kautham_name, pose)

    kautham.kSetRobControlsNoQuery(node, "controls/right_ur3_with_gripper.cntr")
    kautham.kSetQuery(node, ROBOT_HOME_CONTROLS, [])
    ktmpb.ktmpbMoveRobot(node, controls=ROBOT_HOME_CONTROLS, sample_type="init")

    taskfile_name = "taskfile_chess_game.xml" if include_objects else "taskfile_chess_game_no_objects.xml"
    taskfile_path = os.path.join(scenario_folder_path, taskfile_name)
    info = ktmpb.knowledge()
    info.taskfile = open(taskfile_path, "w+")
    info.taskfile.write('<?xml version="1.0"?>\n')
    info.taskfile.write(f'<Task name="{KAUTHAM_PROBLEM_FILE}">\n\t<Initialstate>\n')
    for kautham_name, pose in object_world_poses.items():
        pos_str = " ".join(str(x) for x in pose)
        info.taskfile.write(f'\t\t<Object object="{kautham_name}"> {pos_str} </Object>\n')
    info.taskfile.write("\t</Initialstate>\n")

    for line in _drop_redundant_home_moves(combined_plan):
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
    try:
        simplify_taskfile(taskfile_path)
    except ET.ParseError as exc:
        # A failed action mid-Transfer leaves an unclosed tag. The real error is
        # already logged above; skip simplification rather than pile an XML
        # traceback on top.
        node.get_logger().error(f"Taskfile malformed (an action above probably failed mid-Transfer) - skipping simplification: {exc}")

    # Save the symbolic plan (the lines actually executed) in plans/, for
    # manual review before running the robot.
    plans_dir = os.path.normpath(os.path.join(FILE_DIR, "..", "..", "plans"))
    os.makedirs(plans_dir, exist_ok=True)
    plan_path = os.path.join(plans_dir, os.path.splitext(taskfile_name)[0] + ".plan.txt")
    with open(plan_path, "w") as f:
        f.write("# Plan simbolico ejecutado (MOVE/PICK/PLACE), en orden.\n")
        for plan_line in _drop_redundant_home_moves(combined_plan):
            f.write(plan_line + "\n")

    node.get_logger().info(f"Plan saved in {plan_path}")
    node.get_logger().info(f"Results saved in {taskfile_path}")
    node.destroy_node()
    rclpy.shutdown()


def load_game_file(path):
    """Parse a game file: 'SQUARE=PIECE_NAME' lines set the initial board, other
    non-comment lines are UCI moves in order, '#' starts a comment. Piece names
    must be PEON_NEGRO/PEON_BLANCO (the scene's only two objects). Returns
    (board, moves)."""
    board, moves = {}, []
    with open(path) as f:
        for line in f:
            line = line.split("#", 1)[0].strip()
            if not line:
                continue
            if "=" in line:
                square, piece = line.split("=", 1)
                board[square.strip()] = piece.strip()
            else:
                moves.append(line)
    return board, moves


if __name__ == "__main__":
    # Needs the downward_server and kautham_ros node running (see README).
    #   python3 run_game.py path/to/game.txt [--no-objects]
    # With no file, falls back to a built-in 2-move demo.
    include_objects = "--no-objects" not in sys.argv
    game_file_args = [a for a in sys.argv[1:] if a != "--no-objects"]
    if game_file_args:
        board, moves = load_game_file(game_file_args[0])
        piece_to_kautham = {p: p for p in board.values()}
    else:
        board = {"e4": "e4_piece", "e6": "e6_piece"}
        moves = ["e4e5", "e6e5"]  # advance, then capture on e5
        piece_to_kautham = DEFAULT_PIECE_TO_KAUTHAM

    combined_plan, locations, pieces, final_board = build_combined_plan(board, moves)

    print(f"=== combined plan ({len(combined_plan)} lines) ===")
    for line in combined_plan:
        print(" ", line)
    print(f"\n=== {len(locations)} unique locations touched ===", list(locations.keys()))
    print(f"=== {len(pieces)} unique pick/place actions needed ===")
    for action_type, piece, loc in pieces.values():
        print(f"  {action_type} {piece} @ {loc.name}")
    print("\n=== final board state ===", final_board)

    # models_folder_path="" on purpose: the scene XML's model paths are absolute,
    # and kOpenProblem prepends this folder, so a non-empty one would double it.
    print("\n=== generating taskfile via Kautham (needs kautham_ros node running) ===")
    run_on_kautham(combined_plan, locations, pieces,
                    models_folder_path="",
                    scenario_folder_path=FILE_DIR,
                    piece_to_kautham=piece_to_kautham,
                    include_objects=include_objects)
