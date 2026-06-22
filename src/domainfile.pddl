(define (domain chesscapture)

(:types obstacle robot location)

(:predicates
      (at ?rob - robot ?from - location)
      (handEmpty)
      (holding ?rob - robot ?obs - obstacle)
      (in ?obs - obstacle ?from - location)
      (clear ?loc - location)
      (valid_zone ?loc - location)
      (is_home ?loc - location)
      (is_hover ?loc - location)                  
      (above ?hover - location ?board - location) 
)

(:action move
   :parameters (?rob - robot ?from - location ?to - location)
   :precondition  (and  (at ?rob ?from)
                        (or 
                            ; 1. FASE DE IDA (Aproximación): De Home al punto elevado (Hover)
                            (and (is_home ?from) (is_hover ?to))
                            
                            ; 2. FASE DE AGARRE (Bajar): Del punto elevado a su casilla exacta
                            (above ?from ?to)
                            
                            ; 3. FASE DE VUELTA (Retirada): De la casilla directamente a Home
                            (and (valid_zone ?from) (is_home ?to))
                        )
                  )
   :effect  (and  (at ?rob ?to)
                  (not (at ?rob ?from))
            )
)

(:action pick
   :parameters (?rob - robot ?obs - obstacle ?from - location)
   :precondition  (and  (handEmpty)
                        (in ?obs ?from)
                        (at ?rob ?from)
                  )
   :effect  (and  (holding ?rob ?obs)
                  (not (handEmpty))
                  (not (in ?obs ?from)) 
                  (clear ?from)         
            )
)

(:action place
   :parameters (?rob - robot ?obs - obstacle ?to - location)
   :precondition  (and  (holding ?rob ?obs)
                        (at ?rob ?to)
                        (clear ?to)
                        (valid_zone ?to)
                  )
   :effect  (and  (handEmpty)
                  (in ?obs ?to)
                  (not (holding ?rob ?obs))
                  (not (clear ?to))
            )
)

)
