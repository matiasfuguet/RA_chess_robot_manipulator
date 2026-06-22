# chess_manipulator

Pipeline TAMP para que un UR3e juegue una partida de ajedrez moviendo peones:
planificación lógica (Fast Downward) + planificación geométrica (Kautham / OMPL
RRTConnect) → taskfile → ejecución en el robot real.

A partir de una lista de movimientos UCI se resuelve cada uno por separado (el
dominio de manipulación no sabe de reglas de ajedrez), se concatenan los planes
y se genera un taskfile con las trayectorias articulares. Las capturas se
expanden en dos episodios: la pieza capturada va al graveyard y la pieza que
captura ocupa la casilla. El robot vuelve a HOME entre cada acción.

## Requisitos

Hacen falta dos servicios en marcha (en terminales aparte, ya sourceadas con
`source /opt/ros/jazzy/setup.bash && source ~/ws_tamp/install/setup.bash`):

```bash
# Planificador lógico
~/ws_tamp/install/downward_ros2/lib/downward_ros2/downward_server

# Planificador geométrico (Kautham)
QT_QPA_PLATFORM=xcb ~/ws_tamp/install/kautham_ros/lib/kautham_ros/kautham_ros_node
```

## 1. Generar el taskfile (`run_game.py`)

```bash
python3 run_game.py example_game.txt --no-objects
```

Genera `taskfile_chess_game_no_objects.xml` y, en `../../plans/`, el plan simbólico
ejecutado (`MOVE`/`PICK`/`PLACE`, en orden) para revisarlo antes de tocar el robot.
La opción `--no-objects` aparca las piezas lejos del robot en la escena de Kautham
en lugar de ponerlas en su casilla real: los valores articulares de
`square_to_joints.py` están calibrados para el robot real, no para el modelo
UR3+robotiq_85 de Kautham, así que con las piezas en su sitio la pose de agarre
simulada chocaría con la malla del peón. Es el modo que se usa para el robot real.

### Formato del fichero de partida

```
# Tablero inicial: CASILLA=PIEZA  (solo PEON_BLANCO / PEON_NEGRO)
e2=PEON_BLANCO
d5=PEON_NEGRO

# Movimientos UCI, uno por línea
e2e4
d5e4
```

Los nombres de pieza deben ser `PEON_BLANCO`/`PEON_NEGRO`: son los dos únicos
objetos definidos en la escena de Kautham. Las casillas se limitan a
`REACHABLE_RANKS` (filas 2-8). El pipeline no valida la legalidad de los
movimientos, solo los ejecuta.

## 2. Ejecutar en el robot real (`robot/mover_robot_simplificado.py`)

Copia la carpeta `robot/` (con `pinza10UR3.py`/`pinza40UR3.py`) y el taskfile al
PC del robot — ver `robot_pc_files.zip` — y ejecútalo desde ahí. Antes de
conectar, escribe en `plans/robot_plan_preview.txt` cada `movej` y cada apertura/
cierre de pinza que va a mandar, para poder revisarlo sin haber movido el robot
todavía. Luego lee el taskfile, manda cada punto por el socket (puerto 30002) y
abre/cierra la pinza entre Transit y Transfer. Solo depende de la stdlib.

## 3. Vista previa en simulación

Para revisar una secuencia en Kautham antes de tocar el robot:

```bash
# Demo e4-captura-d5 con la cinemática calibrada para sim
python3 run_kautham_demo.py --no-objects
```

O cargar un taskfile en la GUI:

```bash
QT_QPA_PLATFORM=xcb kautham-gui
```

1. **File → Open Problem** → `OMPL_RRTConnect_chess_pawn_capture.xml`
2. **TAMP → Load Taskfile** → el taskfile generado
3. **Play**

## Estructura

```
chess_manipulator/
├── run_game.py                  # pipeline principal (UCI -> taskfile)
├── square_to_joints.py          # cinemática + geometría del tablero (robot REAL)
├── taskfile_simplify.py         # submuestrea los waypoints del taskfile
├── run_kautham_demo.py          # demo original para visualización en sim
├── kautham_square_to_joints.py  # cinemática calibrada para el modelo de Kautham
├── example_game.txt             # partida de ejemplo
├── ff-domains/                  # dominio y problema PDDL
├── controls/                    # fichero de controles cinemáticos del UR3
└── OMPL_RRTConnect_chess_pawn_capture*.xml  # escenas de Kautham
```

> El robot real se controla desde `robot/mover_robot_simplificado.py`, con las
> pinzas `robot/pinza10UR3.py`/`robot/pinza40UR3.py`. Las posiciones taught y la
> calibración están en `docs/posiciones_reales.md`. Ver la estructura completa
> del repo en el README de la raíz.
