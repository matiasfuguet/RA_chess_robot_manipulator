# Changes and Running Instructions

This file documents all changes made to the project after the initial setup, and provides updated instructions for running the simulation. For the original setup and workspace compilation steps, see `README.md`.

---

## Summary of Changes

### 1. Fixed Kautham Problem File (`OMPL_RRTConnect_chess_pawn_capture.xml`)

**Problem:** kautham-gui crashed or failed to load the file because it could not find the robot and obstacle models.

**What changed:**
- All model paths changed from relative paths (`/robots/...`, `/obstacles/...`) to **full absolute paths** pointing to `/usr/share/kautham/demos/models/`. This removes the need to manually configure a models folder in kautham-gui.
- Controls path changed to `./controls/right_ur3_with_gripper.cntr` (relative to the problem file, as kautham expects).
- Piece positions updated (see section 2 below).

**Why:** kautham-gui's Open Problem dialog does not have a models-folder field in the current installation, so relative paths failed silently.

---

### 2. Corrected Piece Positions for Simulation

**Problem:** The obstacle positions in the Kautham XML and tampconfig were copied directly from real robot TCP measurements (in the robot's own coordinate frame, where Z ≈ −0.36 m means "below the robot base"). This placed the pieces 36 cm below the chessboard surface in simulation.

**What changed:**

The positions were recomputed by:
1. Running forward kinematics on the `GraspControls` joint angles to obtain the TCP position in robot frame.
2. Applying the robot-to-world coordinate transform (robot is at world X = 0.37, rotated −π/2 around Z).
3. Subtracting a calibrated gripper-to-piece offset derived from the reference chess demo.

| Piece | Old (robot TCP frame) | New (Kautham world frame) |
|---|---|---|
| PEON_NEGRO (d5) | X=−0.052 Y=−0.321 Z=−0.358 | X=0.058 Y=0.053 Z=0.060 |
| PEON_BLANCO (e4) | X=0.011 Y=−0.382 Z=−0.360 | X=0.003 Y=−0.003 Z=0.057 |

These positions are used in `OMPL_RRTConnect_chess_pawn_capture.xml` and `tampconfig_chess.xml`.

---

### 3. Two Separate tampconfig Files

Because the piece positions for simulation (Kautham world frame) differ from the real robot execution positions (robot TCP frame), two tampconfig files now exist:

| File | Use | Piece positions |
|---|---|---|
| `tampconfig_chess.xml` | Kautham simulation | Kautham world frame (corrected) |
| `tampconfig_chess_real.xml` | Real UR3e execution | Robot TCP frame (original measurements) |

The `<Actions>` block (all `GraspControls` and `MoveControls` joint angles) is **identical** in both files — only the `<States>` block differs.

---

### 4. Known Limitation — TAMP with Corrected Positions

**Current issue:** Re-running the TAMP pipeline (`ktmpb_full.launch.py`) with `tampconfig_chess.xml` (corrected positions) fails. The OMPL planner cannot find a collision-free path for the MOVE HOME→D5 action because the piece is now correctly sitting at the d5 square, blocking the direct approach.

**Workaround for now:** The `taskfile_tampconfig_chess.xml` included in the repo was generated with the original (below-board) piece positions. It can be used directly in kautham-gui to visualise the full motion sequence. The pieces will appear at the correct on-board positions visually (from the `<Initialstate>` block), but the planned paths were computed without collision with the pieces.

**Planned fix:** A pre-grasp approach (move to 10 cm above the piece, then descend vertically) will be implemented later. This will allow TAMP to plan collision-free paths with pieces on the board, and also improve real robot execution.

---

## How to Run

### kautham-gui — Visualise the Full Task

> **Important:** Launch kautham-gui with `QT_QPA_PLATFORM=xcb` to avoid a Wayland crash when loading the problem file.

```bash
cd ~/GIA/RA/practicaFinal2/RA_chess_robot_manipulator/src/chess_manipulator
QT_QPA_PLATFORM=xcb kautham-gui
```

Once open:
1. **File → Open Problem** → select `OMPL_RRTConnect_chess_pawn_capture.xml`
2. **TAMP → Load Taskfile** → select `taskfile_tampconfig_chess.xml`
3. Press **Start Move** to step through each motion segment one at a time.

---

### Run the Logical Planner Only (Fast Downward)

From the workspace root (`~/ws_tamp`):

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
fast-downward \
  src/chess_manipulator/ff-domains/domain_chess.pddl \
  src/chess_manipulator/ff-domains/problem_chess.pddl \
  --evaluator "hff=ff()" \
  --search "lazy_greedy([hff], preferred=[hff])"
```

Expected output plan (`sas_plan`):
```
(move ur3a home e4)
(pick ur3a peon_blanco e4)
(move ur3a e4 home)
(move ur3a home d5)
(place ur3a peon_blanco d5)
(pick ur3a peon_negro d5)
(move ur3a d5 home)
(move ur3a home graveyard)
(place ur3a peon_negro graveyard)
```

---

### Re-run the Full TAMP Pipeline (regenerate taskfile)

> **Note:** This currently requires using `tampconfig_chess_real.xml` (original piece positions) because the corrected positions cause OMPL to fail. See Known Limitation above.

From `~/ws_tamp`:

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch ktmpb_client ktmpb_full.launch.py \
  models_folder_path:=/usr/share/kautham/demos/models \
  scenario_folder_path:=$(pwd)/../GIA/RA/practicaFinal2/RA_chess_robot_manipulator/src/chess_manipulator \
  tamp_config_filename:=tampconfig_chess_real.xml
```

This opens two xterm windows (Kautham server + Fast Downward server) and runs the TAMP client. When finished, `taskfile_tampconfig_chess.xml` is written to the `chess_manipulator` folder.

---

## File Overview

```
chess_manipulator/
├── ff-domains/
│   ├── domain_chess.pddl               # PDDL domain (unchanged)
│   └── problem_chess.pddl              # PDDL problem (unchanged)
├── controls/
│   └── right_ur3_with_gripper.cntr     # UR3 kinematics (unchanged)
├── launch/
│   └── chess_pawn_capture.launch.py    # ROS 2 launch (references missing ktmpb_base.launch.py — use ktmpb_full.launch.py directly instead)
├── OMPL_RRTConnect_chess_pawn_capture.xml   # Kautham scene (absolute model paths, corrected piece positions)
├── tampconfig_chess.xml                # TAMP config — simulation (corrected piece positions)
├── tampconfig_chess_real.xml           # TAMP config — real UR3e (original TCP positions)  ← NEW
├── taskfile_tampconfig_chess.xml       # Pre-computed motion paths (load this in kautham-gui)
├── README.md                           # Original setup and compilation instructions
└── CHANGES.md                          # This file
```
