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
    
    :precondition (and (at ?rbt ?from) 
    			(not (at ?rbt ?to)))
    			
    :effect (at ?rbt ?to)
  )

  (:action pick
    :parameters(?rbt - robot ?rm - room ?itm - item)
    
    :precondition (and (hand_empty ?rbt) 
    			(at ?rbt ?rm)
    			(in ?itm ?rm))
    			
    :effect (and (not (in ?itm ?rm))
    		   (holding ?rbt ?itm))
  )

  (:action place
    :parameters(?rbt - robot ?rm - room ?itm - item)
    
    :precondition (and (holding ?rbt ?itm) 
    			(at ?rbt ?rm) 
    			)
    			
    :effect (and (in ?itm ?rm)
    		  (hand_empty ?rbt))
  )
)
