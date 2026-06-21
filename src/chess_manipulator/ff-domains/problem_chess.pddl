(define (problem chess_pawn_capture)

(:domain chesscapture)

(:objects
    e4 d5 graveyard home e4_hover d5_hover graveyard_hover - location
    peon_negro peon_blanco - obstacle
    ur3a - robot
)

(:init
    (in peon_negro d5)
    (in peon_blanco e4)
    (at ur3a home)
    (handEmpty)
    (clear graveyard)
    (clear home)
    (is_home home) 

    (is_hover e4_hover)
    (is_hover d5_hover)
    (is_hover graveyard_hover)
    
    (above e4_hover e4)
    (above d5_hover d5)
    (above graveyard_hover graveyard)
    
    (valid_zone e4)
    (valid_zone d5)
    (valid_zone graveyard)
)

(:goal
    (and  (in peon_negro graveyard)
          (in peon_blanco d5)
          (at ur3a home)
    )
)

)
