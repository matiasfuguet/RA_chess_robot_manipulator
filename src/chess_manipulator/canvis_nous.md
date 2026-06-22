# Cambios nuevos

Este documento describe los ficheros Python nuevos añadidos para generalizar la demo
original (captura única e4xd5) a una *lista arbitraria de movimientos de ajedrez*,
introducidos a partir del fichero `e4xd5`. La idea central: en vez de tener un único
`tampconfig`/`problem.pddl` escritos a mano para un solo movimiento, estos scripts
generan el PDDL y la geometría necesarios para *cada movimiento* de una partida,
sobre la marcha.

---

## 1. `square_to_joints.py`

**Qué hace:** Todo lo necesario para convertir "quiero ir a la casilla X" en
"estos son los 6 ángulos articulares del UR3e", calibrado contra el **robot real**:

- Cinemática directa/inversa del UR3e (cadena cinemática oficial de Kautham, con un
  offset de base calibrado empíricamente contra los puntos reales medidos en
  `posiciones_reales.md`).
- Interpolación de la posición cartesiana de cualquier casilla del tablero o de
  cualquier hueco del "cementerio" (graveyard), a partir de solo 3 puntos calibrados
  (d5, e4 y graveyard fila 5).
- Generación de los fragmentos XML `<Move>`/`<Pick>`/<Place>` que espera el formato
  `tampconfig` de ktmpb.

**Limitaciones conocidas (documentadas en el propio código):**
- Solo las filas 2-8 del tablero son alcanzables (`REACHABLE_RANKS`); la fila 1 no
  converge con el solver de IK actual.
- Solo los huecos 4-8 del graveyard son alcanzables (`GRAVEYARD_REACHABLE_SLOTS`);
  los huecos 1-3 comparten el mismo problema de alcance que la fila 1.

**Cómo probarlo solo (sin ROS):**
```bash
cd src/chess_manipulator
python3 square_to_joints.py
```
Imprime el error de la cinemática directa contra los 3 puntos reales conocidos
(debería ser de pocos milímetros) y un ejemplo de los fragmentos XML generados.

---

## 2. `run_game.py`

**Qué hace:** Orquesta una lista de movimientos UCI (ej. `"e2e4"`) de principio a fin:

1. Por cada movimiento, genera un problema PDDL pequeño (usando `square_to_joints.py`
   para la geometría) — una captura se expande automáticamente en dos episodios
   (la pieza capturada va al graveyard, luego la pieza que captura ocupa la casilla).
2. Resuelve cada problema llamando al servicio ROS real `downward_service` (Fast
   Downward), preservando el orden real de la partida — el dominio PDDL no entiende
   de reglas de ajedrez, así que no se puede resolver como un único problema grande.
3. Concatena los planes de todos los movimientos en una sola secuencia.
4. Genera las acciones `tampconfig` (Move/Pick/Place) necesarias para todas las
   casillas/piezas tocadas en toda la partida, reutilizando sin modificar
   `MOVE.py`/`PICK.py`/`PLACE.py` de `ktmpb_client`.
5. Ejecuta esa secuencia completa contra Kautham (vía `kautham_ros`), escribiendo
   **un único taskfile continuo** para toda la partida.

**Importante:** usa la cinemática de `square_to_joints.py`, calibrada para el
**robot real**. Si una casilla concreta no se ve bien colocada en la simulación de
Kautham, es el problema de calibración ya conocido (ver sección 5) — no afecta a la
ejecución en el robot real.

**Cómo ejecutarlo con un fichero de partida propio:**

1. Arrancar los dos servicios, cada uno en su terminal:
   ```bash
   source /opt/ros/jazzy/setup.bash && source ~/ws_tamp/install/setup.bash
   ~/ws_tamp/install/downward_ros2/lib/downward_ros2/downward_server
   ```
   ```bash
   source /opt/ros/jazzy/setup.bash && source ~/ws_tamp/install/setup.bash
   QT_QPA_PLATFORM=xcb ~/ws_tamp/install/kautham_ros/lib/kautham_ros/kautham_ros_node
   ```
   (`ros2 run` puede no estar disponible en algunos entornos — en ese caso, usar la
   ruta directa al ejecutable como arriba, o instalar `ros-jazzy-ros2run`.)

2. Ejecutar, en una tercera terminal (con el mismo `source` de ROS):
   ```bash
   cd src/chess_manipulator
   python3 run_game.py example_game.txt
   ```
   Si no se da ningún fichero, ejecuta una demo de 2 movimientos integrada.

**Formato del fichero de partida** (ver `example_game.txt`):
```
# Tablero inicial: CASILLA=NOMBRE_PIEZA
e4=PEON_BLANCO
e6=PEON_NEGRO

# Movimientos (UCI, uno por línea)
e4e5
e6e5
```
Los nombres de pieza deben ser `PEON_NEGRO`/`PEON_BLANCO` para que el paso de
generación del taskfile funcione — son los dos únicos objetos reales definidos en la
escena de Kautham (`OMPL_RRTConnect_chess_pawn_capture.xml`). La parte de
generación del plan PDDL no tiene esta restricción.

---

## 3. `kautham_square_to_joints.py`

**Qué hace:** Lo mismo que `square_to_joints.py`, pero calibrado para la
**simulación de Kautham**, no para el robot real. Es un fichero **separado a
propósito**: la escena de Kautham usa el modelo cinemático del UR3 (no UR3e — el
xacro de la escena no especifica `ur_type`, así que usa el valor por defecto `"ur3"`)
y una pinza distinta (robotiq_85 en simulación, frente a la pinza real OnRobot RG2),
así que los mismos ángulos articulares que son correctos en el robot real no colocan
la pinza en el sitio correcto dentro de Kautham.

Calibrado contra los valores de agarre de d5/e4 que ya existían en `tampconfig_chess.xml`
(puestos por el autor original) y las posiciones reales de las piezas en el fichero
de escena. El error resultante es inferior a 0.3mm para ambos puntos.

**Cómo probarlo:**
```bash
cd src/chess_manipulator
python3 kautham_square_to_joints.py
```

---

## 4. `run_kautham_demo.py`

**Qué hace:** Ejecuta la demo *original* de captura e4xd5
(`ff-domains/problem_chess.pddl`, sin generalizar a una lista de movimientos) contra
Kautham, pero usando `kautham_square_to_joints.py` en vez de `square_to_joints.py`.
Sirve para visualizar correctamente la demo en simulación sin tocar
`run_game.py`/`square_to_joints.py`, que deben seguir calibrados para el robot real.

**Cómo ejecutarlo:** igual que `run_game.py` (los mismos dos servicios ROS), y luego:
```bash
cd src/chess_manipulator
python3 run_kautham_demo.py
```

**Modo sin piezas (`--no-objects`):** quita las dos piezas (PEON_NEGRO/PEON_BLANCO)
de la escena de Kautham y ejecuta solo los movimientos `move` del plan (sin pick/place,
que necesitan un objeto real al que agarrarse). Sirve para revisar únicamente las
trayectorias del robot sin que la detección de colisiones de Kautham interfiera con el
agarre exacto de las piezas — ese ajuste fino se deja para el robot real.
```bash
python3 run_kautham_demo.py --no-objects
```

---

## 5. `taskfile_simplify.py`

**Qué hace:** Reduce el número de waypoints `<Conf>` que `MOVE.py`/`PICK.py`/`PLACE.py`
(de `ktmpb_client`, sin modificar) escriben dentro de cada bloque `<Transit>`/`<Transfer>`
del taskfile. RRTConnect devuelve la solución sin simplificar (cientos de puntos muy
próximos entre sí), y reproducir eso punto a punto en `kautham-gui` (sección 4 del
`README.md`) es lo que hacía que el robot tardase mucho en cada movimiento.

La lógica es la misma que ya usaba `mover_robot_simplificado.py` (en la raíz del repo,
pensado para el robot real): quedarse con 1 de cada `step` puntos (por defecto 20) y
asegurar siempre que el punto final exacto se conserva, para no perder precisión en el
destino. Se aplica automáticamente al taskfile generado, justo después de cerrarlo, en
`run_kautham_demo.py` y `run_game.py` — no hace falta ningún paso manual.

**Cómo probarlo solo:**
```python
from taskfile_simplify import simplify_taskfile
simplify_taskfile("taskfile_kautham_demo.xml", step=20)
```

---

## 6. Limitación conocida y pendiente: calibración visual en Kautham

Con `kautham_square_to_joints.py`, la secuencia completa d5 (recoger negro, llevarlo
al graveyard) funciona de principio a fin en Kautham, incluyendo el agarre y la
suelta reales. La secuencia de e4 falla al intentar bajar a por la pieza blanca: la
pinza, en la posición de agarre calculada, toca ligeramente la pieza (probablemente
necesite un pequeño ajuste, sobre todo en el componente Y). Se ha dejado pendiente
para retomar más adelante — no afecta ni a `run_game.py` (robot real) ni a la
generación del plan PDDL, solo a la visualización de e4 específicamente en Kautham.

---

## Resumen de comandos

| Quiero... | Comando |
|---|---|
| Probar la geometría/cinemática del robot real | `python3 square_to_joints.py` |
| Probar la geometría/cinemática de Kautham | `python3 kautham_square_to_joints.py` |
| Generar el plan + taskfile de una partida propia | `python3 run_game.py mi_partida.txt` |
| Ver la demo original e4xd5 en Kautham (visualización corregida) | `python3 run_kautham_demo.py` |
| Ver solo el movimiento del robot en Kautham, sin las piezas (sin colisiones) | `python3 run_kautham_demo.py --no-objects` |
