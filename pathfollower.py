from vector2D import Vector2D as vec
from .steering import Steering


class PathFollowing(Steering):

    def __init__(self, 
        steeringForce: float, 
        maxVel: float, 
        waypoints: List[Position], 
        backtrack:bool = False, 
        rest: bool = False, 
        leewayDistance: int = 80):

        super().__init__(steeringForce, maxVel)

        if not all(map(lambda pos: True if len(pos) == 2 else False, waypoints)):
            raise ValueError(f"all values in the waypoint must be of (x, y) values! \n{waypoints}")

        self.leewayDistance = leewayDistance
        self.backtrack      = backtrack
        self._waypoints     = vec.convert(waypoints)
        self.rest           = rest

        self.__currentNodeCount  = 0
        self.__currentTrajectory = self._waypoints[self.__currentNodeCount]

        self.__resting = False
    

    @property
    def resting(self):
        return self.__resting


    @property
    def waypoints(self):
        return self._waypoints


    def add_node(self, *node) -> None:
        self._waypoints.extend(vec.convert(node))


    def update(self, origin_pos, origin_vel, dt) -> vec:
        if not self.__resting:
            self.check_trajectory()
            self.calculate_desired_vector(origin_pos, self.__currentTrajectory)
            return self.calculate_turning_force(origin_vel) * dt
        else:
            # will enable the approach steering method 
            # if the < rest > option is enabled & the entity has arrived at one of the waypoints
            return self.approach(origin_pos, origin_vel, self.__currentTrajectory) * dt


    def check_trajectory(self) -> None:
        # switch the current trajectory to the next node in the waypoints, 
        # if the distance between the entity and the node is smaller than the leewayDistance

        if self.desired_distance < self.leewayDistance:
            self.__currentNodeCount  = (self.__currentNodeCount + 1) % (len(self._waypoints) - 1)
            self.__currentTrajectory = self._waypoints[self.__currentNodeCount]
            # reverse the waypoints if the entity has reached the end node
            # --> creates the backtracking effect
            if self.__currentNodeCount == 0:
                self._waypoints = self._waypoints[::-1] if self.backtrack else self._waypoints

            if self.rest:
                self.__resting = True


    def __str__(self):
        return pformat(vars(self), indent=4, sort_dicts=False)