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
)

(:action move
   :parameters (?rob - robot ?from - location ?to - location)
   :precondition  (and  (at ?rob ?from)
                        (or (is_home ?from) (is_home ?to)) 
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
