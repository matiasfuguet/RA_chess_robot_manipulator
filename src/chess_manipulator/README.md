# chess_manipulator

Carpeta autónoma con el escenario TAMP de captura de peón de ajedrez con robot UR3.  
El peón negro ocupa d5 y el peón blanco e4. El objetivo es que el blanco capture al negro: el negro va al graveyard y el blanco ocupa d5.

El dominio obliga al robot a regresar a la posición neutra (home) entre cada acción de manipulación, modelado mediante el predicado `(connected)` que restringe los movimientos a pasar siempre por `home`.

---

## Estructura de la carpeta

```
chess_manipulator/
├── ff-domains/
│   ├── domain_chess.pddl            # Dominio PDDL con restriccion home via (connected)
│   └── problem_chess.pddl           # Problema: casillas reales e4/d5/graveyard/home
├── controls/
│   └── right_ur3_with_gripper.cntr  # Fichero de controles cinematicos del UR3
├── launch/
│   └── chess_pawn_capture.launch.py # Script ROS 2 launch del pipeline TAMP
├── OMPL_RRTConnect_chess_pawn_capture.xml  # Escena Kautham (robot + piezas)
├── tampconfig_chess.xml             # Config TAMP: regiones, estados, acciones
└── README.md                        # Este fichero
```

---

## 1. Compilar el workspace

```bash
cd ~/ws_tamp
colcon build --symlink-install
source install/setup.bash
```

Si `ros2 run` no está disponible, instalarlo con:

```bash
sudo apt install ros-jazzy-ros2run
```

---

## 2. Ejecutar solo la Planificación Lógica (Fast Downward)

### Opción A — directamente con fast-downward (más simple)

```bash
fast-downward \
  src/chess_manipulator/ff-domains/domain_chess.pddl \
  src/chess_manipulator/ff-domains/problem_chess.pddl \
  --evaluator "hff=ff()" \
  --search "lazy_greedy([hff], preferred=[hff])"
```

### Opción B — via ROS 2 (servidor + cliente en dos terminales)

**Terminal 1** — arrancar el servidor:
```bash
source /opt/ros/jazzy/setup.bash && source ~/ws_tamp/install/setup.bash
ros2 run downward_ros2 downward_server
```

**Terminal 2** — lanzar el cliente:
```bash
source /opt/ros/jazzy/setup.bash && source ~/ws_tamp/install/setup.bash
ros2 run downward_ros2 downward_client \
  --ros-args \
  -p domain_param:=domain_chess \
  -p problem_param:=problem_chess \
  -p pddl_folder_path_param:=$(pwd)/src/chess_manipulator/ff-domains/
```

> El servidor debe estar en marcha antes de lanzar el cliente. El ejecutable se llama `downward_client`, no `downward_node`.

El planificador genera el plan simbólico por stdout. La secuencia esperada es:

```
move ur3a home e4
pick ur3a peon_blanco e4
move ur3a e4 home
move ur3a home d5
place ur3a peon_blanco d5
pick ur3a peon_negro d5
move ur3a d5 home
move ur3a home graveyard
place ur3a peon_negro graveyard
```

El plan también se guarda en el fichero `sas_plan` del directorio de trabajo.

---

## 3. Ejecutar el pipeline TAMP completo (Lógica + Geometría Kautham)

### Opción A — via ROS 2 launch (recomendado)

```bash
source /opt/ros/jazzy/setup.bash && source ~/ws_tamp/install/setup.bash
ros2 launch src/chess_manipulator/launch/chess_pawn_capture.launch.py
```

El script detecta automáticamente la ruta de `tampconfig_chess.xml` relativa a su propia ubicación.

### Opción B — via ktmpb_client directamente

```bash
source /opt/ros/jazzy/setup.bash && source ~/ws_tamp/install/setup.bash
ros2 run ktmpb_client ktmpb_client \
  --ros-args \
  -p config:=$(pwd)/src/chess_manipulator/tampconfig_chess.xml
```

El cliente invoca iterativamente al planificador lógico y al planificador geométrico (OMPL RRTConnect) para cada par de regiones definido en `<Actions>`. El resultado se guarda como `taskfile_tampconfig_chess.xml` junto al tampconfig.

---

## 4. Validación visual en kautham-gui

1. Abre la interfaz gráfica:
   ```bash
   kautham-gui
   ```
2. Menú **File → Open Problem** y selecciona:
   ```
   src/chess_manipulator/OMPL_RRTConnect_chess_pawn_capture.xml
   ```
3. Menú **TAMP → Load Taskfile** y selecciona:
   ```
   src/chess_manipulator/taskfile_tampconfig_chess.xml
   ```
4. Pulsa **Play** para reproducir la secuencia completa de movimientos del robot.
