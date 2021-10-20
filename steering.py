from vector2D import Vector2D as vec
from math import cos, sin, radians
from typing import List, Tuple
'''
TODO: 
A* PATHFINDING 
--> ViewBox: trigger algorithm when player collides with the viewbox
--> remember player's last position
'''
# useful:  method_list = [func for func in dir(Foo) if callable(getattr(Foo, func))]
Position = Tuple[int, int]


class Steering:


    def __init__(self, steeringForce: float, maxVel: float, approach_distance: int = 0, halt_distance: int = 0):

        self.steeringForce = steeringForce # steeringForce: lower --> makes the movement smooth, but slow
        self.maxVel        = maxVel

        self.halt_distance     = halt_distance
        self.approach_distance = approach_distance

        self.desired_distance = float('inf')
        self.desired          = vec(0, 0)


    def calculate_desired_vector(self, origin_pos: Position, target_pos: Position) -> None:
        self.desired.update(target_pos - origin_pos)
        self.desired_distance = (target_pos - origin_pos).length 
        self.desired.normalize_ip()


    def calculate_turning_force(self, origin_vel: vec) -> vec:
        # difference: the vector of entity's current trajectory vs the vector of desired trajectory
        difference = (self.desired * self.maxVel) - origin_vel

        # if the difference is greater than the lerp factor, 
        # the entity's current trajectory does not equal to the desired direction
        if difference.length > self.steeringForce:
            # decrease the turning speed of the entity
            difference.scale_to_length(self.steeringForce)
        # this will slowly (depends on the lerp factor),
        # turn the current vector/trajectory of the entity towards the desired direction
        # will be zero when the entity is heading towards the direction of the target
        # --> no more turnings
        return difference


    def seek(self, origin_pos: Position, origin_vel: vec, target_pos: Position) -> vec:
        self.calculate_desired_vector(origin_pos, target_pos)
        if self.desired_distance < self.halt_distance:
            self.desired *= 0
        return self.calculate_turning_force(origin_vel)


    def approach(self, origin_pos: Position, origin_vel: vec, target_pos: Position) -> vec:
        self.calculate_desired_vector(origin_pos, target_pos)
        if self.desired_distance < self.approach_distance:
            self.desired *= (self.desired_distance / self.approach_distance) if self.desired_distance > self.halt_distance else 0
        return self.calculate_turning_force(origin_vel)


    def pursue(self, origin_pos: Position, origin_vel: vec, target_pos: Position, target_vel: vec) -> vec:
        return self.seek(origin_pos, origin_vel, target_pos + target_vel)


    def evade(self, origin_pos: Position, origin_vel: vec, target_pos: Position, target_vel: vec) -> vec:
        return ( self.pursue(origin_pos, origin_vel, target_pos, target_vel) * -1 )


    def __str__(self):
        return pformat(vars(self), indent=4, sort_dicts=False)