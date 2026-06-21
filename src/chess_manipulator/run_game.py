"""Iterates a list of UCI chess moves, builds + solves each one's PDDL
problem in turn (preserving real game order - the manipulation-only
domain has no concept of chess legality, so this can't be solved as one
big PDDL problem), concatenates the resulting plan lines, then feeds the
whole thing through ktmpb's existing MOVE/PICK/PLACE loop as one
continuous sequence instead of replaying a single hard-coded demo.

Needs `ros2 run downward_ros2 downward_server` running for build_combined_plan(),
and the Kautham ROS node running too for run_on_kautham() (see __main__).
"""

import os
import sys
import xml.etree.ElementTree as ET

FILE_DIR = os.path.dirname(os.path.abspath(__file__))
# ktmpb_client's modules (MOVE.py/PICK.py/PLACE.py/ktmpb_python_interface.py)
# are bare-imported by that package itself via a sys.path.append of its own
# directory, so we need the same directory on our path to import them the
# same way (not a ROS package import).
KTMPB_CLIENT_DIR = os.path.expanduser(
    "~/ws_tamp/src/task_and_motion_planning2/ktmpb/ktmpb_client/ktmpb_client"
)
for _path in (FILE_DIR, KTMPB_CLIENT_DIR):
    if _path not in sys.path:
        sys.path.append(_path)

import square_to_joints as sj

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
    """board: dict square->piece_name (current state, before this move).
    A capture expands into two manipulation episodes (captured piece ->
    graveyard, then capturing piece -> destination), mirroring the
    existing hand-written problem_chess.pddl. Returns dict with:
    pddl_text, new_board, new_used_graveyard_slots, locations (list of
    Location, for tampconfig generation), pieces (list of (action_type,
    piece_name, Location) needing Pick/Place actions).

    Scope: regular moves and captures only - no castling, en passant or
    promotion. Squares are restricted to square_to_joints.REACHABLE_RANKS
    (2-8) until rank 1 reachability is investigated further."""
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
        init_facts.append(f"(clear {dst_loc.name})")  # dst starts empty, needed for place

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
    """board: dict square->piece_name (starting state). moves: list of
    UCI strings, e.g. ["e2e4", "d7d5"]. Returns (combined_plan_lines,
    locations, pieces, final_board) - locations/pieces deduplicated by
    name, ready for build_actions_list().

    Pass an existing downward_client to reuse one rclpy session across
    several calls (e.g. alongside run_on_kautham); otherwise this manages
    its own rclpy.init()/shutdown()."""
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
    """Generates the tampconfig Move/Pick/Place Action entries needed for
    everything touched across the whole move list, parsed through the
    *unmodified* Move_read/Pick_read/Place_read from ktmpb_client so the
    resulting dicts are exactly the shape MOVE.py/PICK.py/PLACE.py expect.

    piece_to_kautham: dict mapping each logical piece name (e.g.
    "e4_piece") to the real Kautham scene object it represents (e.g.
    "PEON_BLANCO") - see OMPL_RRTConnect_chess_pawn_capture.xml."""
    import MOVE
    import PICK
    import PLACE

    # Seed progressively from one location to the next (rather than
    # resetting to the same static seed each time) - this is what made IK
    # converge reliably across the whole board/graveyard; a fixed distant
    # seed can fail to converge even within REACHABLE_RANKS.
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


# Known world-frame poses (Kautham frame, axis-angle) for the scene's two
# pre-defined pieces - see OMPL_RRTConnect_chess_pawn_capture.xml. A game
# using different/more pieces needs that scene file extended with matching
# <Obstacle> entries first; this script doesn't generate those.
DEFAULT_OBJECT_WORLD_POSES = {
    "PEON_NEGRO": [0.058, 0.053, 0.060, 0.013345, -0.999580, 0.025736, 3.147323],
    "PEON_BLANCO": [0.003, -0.003, 0.057, 0.012559, -0.999036, 0.042071, 3.185071],
}
DEFAULT_PIECE_TO_KAUTHAM = {"e4_piece": "PEON_BLANCO", "e6_piece": "PEON_NEGRO"}
KAUTHAM_PROBLEM_FILE = "OMPL_RRTConnect_chess_pawn_capture.xml"
ROBOT_HOME_CONTROLS = [0.500, 0.375, 0.500, 0.375, 0.500, 0.500, 0.813]


def run_on_kautham(combined_plan, locations, pieces, models_folder_path,
                    scenario_folder_path, piece_to_kautham=None,
                    object_world_poses=None, show_rviz=False):
    """Replays the combined plan through Kautham. Requires the kautham_ros
    ROS 2 node already running (see __main__ below)."""
    import rclpy
    from rclpy.node import Node
    import kautham_ros.kautham_ros_interface_python as kautham
    import ktmpb_python_interface as ktmpb
    import MOVE  # noqa: F401  (needed in globals() for ktmpb's dispatch-by-tag)
    import PICK  # noqa: F401
    import PLACE  # noqa: F401

    piece_to_kautham = piece_to_kautham or DEFAULT_PIECE_TO_KAUTHAM
    object_world_poses = object_world_poses or DEFAULT_OBJECT_WORLD_POSES
    actions_list = build_actions_list(locations, pieces, piece_to_kautham)

    rclpy.init()
    node = Node("chess_game_runner")
    node.show_rviz = show_rviz

    kautham.kOpenProblem(node, models_folder_path, os.path.join(scenario_folder_path, KAUTHAM_PROBLEM_FILE))
    for kautham_name, pose in object_world_poses.items():
        kautham.kSetObstaclePos(node, kautham_name, pose)

    kautham.kSetRobControlsNoQuery(node, "controls/right_ur3_with_gripper.cntr")
    kautham.kSetQuery(node, ROBOT_HOME_CONTROLS, [])
    ktmpb.ktmpbMoveRobot(node, controls=ROBOT_HOME_CONTROLS, sample_type="init")

    taskfile_path = os.path.join(scenario_folder_path, "taskfile_chess_game.xml")
    info = ktmpb.knowledge()
    info.taskfile = open(taskfile_path, "w+")
    info.taskfile.write('<?xml version="1.0"?>\n')
    info.taskfile.write(f'<Task name="{KAUTHAM_PROBLEM_FILE}">\n\t<Initialstate>\n')
    for kautham_name, pose in object_world_poses.items():
        pos_str = " ".join(str(x) for x in pose)
        info.taskfile.write(f'\t\t<Object object="{kautham_name}"> {pos_str} </Object>\n')
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
    node.get_logger().info(f"Results saved in {taskfile_path}")
    node.destroy_node()
    rclpy.shutdown()


def load_game_file(path):
    """Parses a simple game file: lines with '=' set the initial board
    (SQUARE=PIECE_NAME), other non-comment lines are UCI moves in order.
    '#' starts a comment. Returns (board, moves). For the taskfile-
    generation step to work, piece names must be PEON_NEGRO/PEON_BLANCO -
    the only 2 real objects currently in the Kautham scene."""
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
    # Requires `ros2 run downward_ros2 downward_server` running in another
    # (sourced) terminal, and `ros2 run kautham_ros kautham_ros_node` in a
    # third for the taskfile-generation step.
    #   python3 run_game.py path/to/game.txt
    # falls back to the built-in 2-move demo if no file is given.
    if len(sys.argv) > 1:
        board, moves = load_game_file(sys.argv[1])
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

    # Needs the live Kautham ROS node running too (separate terminal):
    #   ros2 run kautham_ros kautham_ros_node
    # models_folder_path="" on purpose - OMPL_RRTConnect_chess_pawn_capture.xml's
    # model paths are already absolute; kOpenProblem prepends whatever
    # folder you pass here, so a non-empty one double-prefixes them and
    # the open fails outright.
    print("\n=== generating taskfile via Kautham (needs kautham_ros_node running) ===")
    run_on_kautham(combined_plan, locations, pieces,
                    models_folder_path="",
                    scenario_folder_path=FILE_DIR,
                    piece_to_kautham=piece_to_kautham)
