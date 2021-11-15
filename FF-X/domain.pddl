(define (domain turtlebot3-domain)

  (:types
    room robot item
  )

  (:predicates
    (at ?rbt - robot ?rm - room)
    (in ?itm - item ?rm - room)
    (holding ?rbt - robot ?itm - item)
    (hand_empty ?rbt - robot)
  )

  (:action move
    :parameters (?rbt - robot ?from - room ?to - room)
    :precondition (at ?rbt - robot ?rm - room)
    :effect (in ?itm - item ?rm - room)
  )

  (:action pick
    :parameters(?rbt - robot ?rm - room ?itm - item)
    :precondition (and (hand_empty ?rbt - robot))
    :effect (holding ?rbt - robot ?itm - item)
  )

  (:action place
    :parameters(?rbt - robot ?rm - room ?itm - item)
    :precondition (holding ?rbt - robot ?itm - item)
    :effect (hand_empty ?rbt - robot)
  )
)
