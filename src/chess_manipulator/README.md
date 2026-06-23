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
# Tablero inicial: CASILLA=PIEZA
e2=PEON_BLANCO
d5=PEON_NEGRO

# Movimientos UCI, uno por línea
e2e4
d5e4
```

Los nombres de pieza deben tener un `<KauthamName>` correspondiente en
`OMPL_RRTConnect_chess_pawn_capture.xml` (actualmente: `PEON_BLANCO`,
`PEON_NEGRO`, `CABALLO_BLANCO`, `PEON_BLANCO_2`, `PEON_BLANCO_3`,
`ALFIL_NEGRO` - ver `DEFAULT_OBJECT_WORLD_POSES`/`PARKING_SPOTS` en
`run_game.py`). Una pieza nueva necesita una entrada `<Obstacle>` a juego en
esa escena, o Kautham se cae al intentar adjuntarla. Las casillas se limitan a
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

## Cómo funciona por dentro

### `run_game.py`, paso a paso

El dominio de manipulación (`ff-domains/domain_chess.pddl`) no sabe nada de
ajedrez - solo conoce "robot, recoge obstáculo X de la ubicación Y, deja en Z".
No puede razonar sobre una partida entera de golpe, así que **cada movimiento
se resuelve como su propio problema PDDL independiente**, en orden:

1. **`load_game_file`** lee el fichero de partida: las líneas `CASILLA=PIEZA`
   forman el `board` inicial (un dict casilla→pieza), el resto son movimientos
   UCI en una lista.

2. **`build_combined_plan(board, moves)`** itera los movimientos. Para cada
   uno, **`build_move_episode`**:
   - Separa el movimiento en origen/destino (`parse_uci_move`).
   - Calcula la pose cartesiana de cada casilla con
     `sj.Location.for_square(...)` (geometría pura, sin Kautham ni IK todavía
     - la pose queda guardada como atributo del objeto `Location`).
   - Si la casilla destino ya tiene una pieza, es una captura: añade un
     episodio extra (recoger la pieza capturada, dejarla en el graveyard -
     usando el siguiente slot libre de `GRAVEYARD_REACHABLE_SLOTS`) antes del
     movimiento real.
   - Construye el texto PDDL (`:objects`/`:init`/`:goal`) para ese movimiento
     y lo manda a Fast Downward (`ktmpb.DownwardClient`). La solución (líneas
     `MOVE`/`PICK`/`PLACE`) se concatena en `combined_plan`.
   - El `board` se actualiza para el siguiente movimiento.

   Al final de este bucle, `build_combined_plan` devuelve `combined_plan` (el
   plan simbólico completo, solo nombres, sin geometría todavía), `locations`
   y `pieces` (cada casilla/pieza única tocada en toda la partida, con su
   `Location` ya calculada) y el `board` final.

3. **`build_actions_list(locations, pieces, piece_to_kautham)`** (llamada
   dentro de `run_on_kautham`, no antes) convierte esas `Location` ya
   calculadas en ángulos articulares reales, vía IK encadenada (el resultado
   de una casilla sirve de seed para la siguiente - así se mantiene la
   convergencia en todo el tablero). Construye tres tipos de snippet
   tampconfig (parseados con los `Move_read`/`Pick_read`/`Place_read` de
   ktmpb_client, para que tengan la forma exacta que `MOVE`/`PICK`/`PLACE`
   esperan):
   - 4 `<Move>` por casilla (home↔hover, hover↔casilla, ambos sentidos).
   - 1 `<Move>` hover→hover por cada *par* de casillas usadas en la partida -
     necesario porque el dominio permite transferir directamente entre dos
     hovers sin pasar por home (mientras se lleva una pieza de un pick a su
     place), y no se sabe de antemano qué par concreto elegirá el planificador.
   - 1 `<Pick>`/`<Place>` por cada acción de recogida/colocación necesaria.

   El resultado, `actions_list`, es solo una tabla de consulta en memoria -
   no es un fichero, y Kautham no la carga. Es lo que `run_on_kautham` usa
   para encontrar, por nombre, los datos articulares de cada línea del plan
   simbólico (`ktmpb.find_action_for_plan_line`).

4. **`run_on_kautham(...)`** es donde se solucionan las trayectorias reales y
   se escribe el taskfile:
   - En `--no-objects`, `kAttachObject`/`kDetachObject` quedan parcheados:
     antes de agarrar, la pieza se teletransporta brevemente a su última
     posición conocida (con un offset lateral, `ATTACH_CLEARANCE_X`, para no
     chocar con la pinza simulada cerrada); al soltarla, se aparca de nuevo
     lejos (`PARKING_SPOTS`). Desde el punto de vista de `PICK`/`PLACE` no
     cambia nada - siguen llamando a las mismas funciones.
   - Se abre el taskfile y se escribe su cabecera.
   - El bucle principal recorre `combined_plan` (filtrando con
     `_drop_redundant_home_moves` el tramo "casilla→hover" que ya resuelve
     `PICK`/`PLACE` internamente en su retirada, para no resolverlo dos
     veces). Por cada línea busca su entrada en `actions_list` y llama a la
     función real `MOVE`/`PICK`/`PLACE` de ktmpb_client - **aquí es donde
     Kautham resuelve de verdad con RRTConnect** y escribe el resultado en el
     taskfile como bloques `<Transit>`/`<Transfer>` de `<Conf>`.
   - Al cerrar el taskfile, se llama a `simplify_taskfile(..., checkpoints_only=True)`.
   - Por último se guarda el plan simbólico completo (sin filtrar) en
     `plans/*.plan.txt`, para revisión manual.

### `square_to_joints.py`

Vive aquí la cinemática (FK/IK) y la geometría del tablero del robot REAL,
porque son el mismo trabajo: dada una casilla (`square_pose`, interpolando
`FILE_AXIS`/`RANK_AXIS` desde 3 puntos enseñados), calcula su pose cartesiana,
la de su hover (`HOVER_HEIGHT` más arriba), resuelve IK, y genera los snippets
tampconfig (`tampconfig_move_actions`/`tampconfig_pick_or_place`/
`tampconfig_hover_transfer`). `REACHABLE_RANKS`/`GRAVEYARD_REACHABLE_SLOTS`
son límites de convergencia/planificación encontrados empíricamente, no
límites físicos exactos del robot.

### `taskfile_simplify.py`

Se ejecuta una vez, después de cerrar el taskfile completo - no durante cada
solve individual (eso ya lo hace el propio `_Simplify Solution` de Kautham,
dentro de cada resolución de RRTConnect, *antes* de que el resultado llegue
al taskfile). Hace tres cosas, en orden:

1. **Elimina bloques enteros redundantes**: cuando no se lleva ninguna pieza,
   cada acción escribe su propio `<Transit>` - si `PICK`/`PLACE` resuelve "ir
   al objeto" otra vez (ya resuelto por el `MOVE` explícito anterior), aparece
   como un segundo bloque con el mismo origen/destino exacto. Se borra.
2. **Elimina revisitas dentro de un bloque**: mientras se lleva una pieza,
   ktmpb_client fusiona varias acciones en un solo `<Transfer>` continuo - el
   mismo tipo de redundancia aparece *dentro* de ese bloque como una repetición
   consecutiva o un "rebote" final a un punto ya visitado.
3. **Divide en checkpoints** (`checkpoints_only=True`): cada bloque se separa
   en un sub-bloque por cada tramo real, cortando en cada punto con nombre
   (hover/home/agarre, vía `keep_joints`) - así cada bloque resultante tiene
   exactamente los dos extremos de un solo tramo, y el robot puede reducirlo
   a "solo el primer y el último punto" sin perder ninguna parada de
   seguridad enterrada en medio de un bloque fusionado.

### `run_kautham_demo.py` / `kautham_square_to_joints.py`

Demo separada para visualizar en `kautham-gui` - **no genera el mismo
taskfile que usa el robot real**. La escena de Kautham simula un UR3 (no
UR3e) con pinza robotiq_85 (no la OnRobot RG2 real), así que los mismos
ángulos articulares no caen en el mismo sitio cartesiano en los dos modelos
(~17cm de diferencia). `kautham_square_to_joints.py` reutiliza los mismos
valores articulares ya enseñados del robot real para d5/e4/graveyard, y les
aplica un único offset constante (`BASE_OFFSET`, ajustado una vez) para que
caigan en la posición correcta dentro del modelo de Kautham. Limitaciones
actuales: sigue fijo a la demo original (e4 captura d5), no acepta un fichero
de partida arbitrario como `run_game.py`; y su modo `--no-objects` funciona
distinto al de `run_game.py` - en vez de aparcar las piezas, simplemente
descarta las líneas `PICK`/`PLACE` del plan y solo ejecuta los `MOVE`, así que
nunca llega a mostrar una captura.

### `robot/mover_robot_simplificado.py`

Lee el taskfile final y manda cada `movej` por el socket (puerto 30002),
esperando a que termine antes de mandar el siguiente (sin radio de mezcla -
se probó y rompía la sincronía pinza/posición en hardware real). Abre/cierra
la pinza según la adyacencia de bloques Transit/Transfer en el taskfile -
nada de esto está codificado en el propio taskfile, que solo contiene
trayectorias articulares. `FIRST_MOVE_WAIT_SECONDS` cubre el único caso donde
el script no tiene ninguna referencia de dónde está realmente el robot al
arrancar (todos los demás "primeros puntos de bloque" son repeticiones de
donde acabó el bloque anterior, así que ahí sí se puede asumir que ya está
ahí). Antes de conectar, escribe `plans/robot_plan_preview.txt` con cada
`movej` y cada `PINZA: ABRIR/CERRAR` exactos que va a mandar - generado a
partir del taskfile ya simplificado, no del plan simbólico.

### `plans/` y `docs/`

`plans/*.plan.txt` y `plans/robot_plan_preview.txt` son artefactos de
depuración que ningún script vuelve a leer - solo para revisar a mano antes
de mover el robot. El primero es el plan simbólico (sin números, generado por
`run_on_kautham` desde `combined_plan`); el segundo es la previsualización a
nivel articular (con números y acciones de pinza, generado por
`mover_robot_simplificado.py` desde el taskfile ya simplificado) - no son la
misma información en dos formatos, son dos etapas distintas del pipeline.
`docs/posiciones_reales.md` guarda las posiciones reales enseñadas con el
teach pendant, fuente de verdad para toda la calibración de `square_to_joints.py`.
