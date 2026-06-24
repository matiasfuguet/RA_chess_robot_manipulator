# Cambios nuevos

Notas rápidas sobre los scripts añadidos para pasar de la demo fija `e4xd5` a
una lista de movimientos.

## Scripts principales

- `run_game.py`: lee un fichero de partida, genera un problema PDDL por
  movimiento y crea un único taskfile para Kautham/robot.
- `square_to_joints.py`: calcula poses y joints para el robot real a partir de
  las posiciones enseñadas.
- `run_kautham_demo.py`: mantiene la demo original para revisar la simulación.
- `kautham_square_to_joints.py`: calibración separada para el modelo de Kautham.
- `taskfile_simplify.py`: limpia puntos repetidos del taskfile conservando
  checkpoints como home, hover y agarre.

## Ideas importantes

- Las capturas se hacen en dos pasos: primero se lleva la pieza capturada al
  graveyard y después se mueve la pieza que captura.
- La simulación y el robot real no usan exactamente la misma calibración, por
  eso hay dos ficheros de cinemática.
- Los hovers no se deben eliminar a ciegas: son los puntos que evitan bajar o
  retirarse arrastrando la pieza por el tablero.

## Pendiente

- Unificar mejor la demo y el flujo general si hace falta mantener ambos.
- Revisar solo con pruebas reales cualquier cambio en la simplificación del
  taskfile.
