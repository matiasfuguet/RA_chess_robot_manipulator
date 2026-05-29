(define (problem chess_pawn_capture)

(:domain chesscapture)

(:objects
    e4 d5 graveyard home - location
    peon_negro peon_blanco - obstacle
    ur3a - robot
)

(:init
    (in peon_negro d5)
    (in peon_blanco e4)
    (at ur3a home)
    (handEmpty)
    (connected home d5)
    (connected home e4)
    (connected home graveyard)
    (connected d5 home)
    (connected e4 home)
    (connected graveyard home)
)

(:goal
    (and  (in peon_negro graveyard)
          (in peon_blanco d5)
    )
)

)
