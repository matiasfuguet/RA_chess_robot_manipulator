# RA_chess_robot_manipulator

UR3e jugando ajedrez: planificación PDDL + geométrica (Kautham/OMPL) y ejecución
en el robot real. Detalles del pipeline en
[`src/chess_manipulator/README.md`](src/chess_manipulator/README.md).

## Estructura

```
.
├── src/chess_manipulator/   # pipeline TAMP: PDDL, cinemática, generación de taskfiles
├── robot/                   # lo que se copia al PC del robot (mover_robot_simplificado.py, pinzas)
├── plans/                   # generado - plan simbólico + previsualización de movimientos (gitignored)
├── docs/                    # documentación de referencia (changes, todo, calibración, enunciado)
└── robot_pc_files.zip       # bundle de robot/ + taskfile listo para copiar al PC del robot
```

## Flujo rápido

1. `python3 src/chess_manipulator/run_game.py src/chess_manipulator/example_game.txt --no-objects`
   genera el taskfile y el plan simbólico en `plans/`.
2. Revisa `plans/*.plan.txt`.
3. Copia `robot/` + el taskfile al PC del robot (o usa `robot_pc_files.zip`) y ejecuta
   `mover_robot_simplificado.py` ahí — escribe `plans/robot_plan_preview.txt` antes de
   conectar, para revisar los `movej` exactos antes de mover el brazo.
