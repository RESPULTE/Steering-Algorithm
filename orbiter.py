from vector2D import Vector2D as vec
from .steering import Steering



class Orbiting(Steering):


    def __init__(self, 
        steeringForce: float, 
        maxVel: float, 
        orbitalRadius: float, 
        orbitalNodeNum: int, 
        nodeRotationDegree: int = 0, 
        leewayDistance: int = 100):

        super().__init__(steeringForce, maxVel)

        if (not orbitalNodeNum and orbitalRadius) or (orbitalRadius and not orbitalNodeNum):
            raise Exception("Expecting both orbitalRadius and orbitalNodeNum!")

        self.leewayDistance       = leewayDistance
        self.orbitalRadius        = orbitalRadius
        self.nodeRotationDegree   = nodeRotationDegree
        self._orbitalNodeNum      = orbitalNodeNum
        self._waypoints           = []

        self.__angleSegment           = 360 // orbitalNodeNum
        self.__previousTargetPosition = None
        self.__currentTrajectory      = None
        self.__currentNodeCount       = 0
        self.__updated                = False


    @property
    def orbitalNodeNum(self):
        return self._orbitalNodeNum
        

    @orbitalNodeNum.setter
    def orbitalNodeNum(self, num):
        self._orbitalNodeNum = num  
        self.__angleSegment  = 360 // num
        self.__updated       = True
 

    def update(self, origin_pos, origin_vel, target_pos, dt) -> vec:
        if target_pos != self.__previousTargetPosition or self.__updated:
            # will be activated if, and only if the target that is being orbited, changes its position
            self.__previousTargetPosition = target_pos
            self.recalculate_trajectory(target_pos)
            self.__updated = False
        self.check_trajectory(origin_pos)
        self.calculate_desired_vector(origin_pos, self.__currentTrajectory)
        return self.calculate_turning_force(origin_vel) * dt


    def recalculate_trajectory(self, target_pos) -> None:
        self._waypoints.clear()
        # divide a circle by the predetermined segment of angles
        # scale the calculated x and y position with
        temp_waypoints = []
        for angle in range(0, 360, self.__angleSegment):
            radian = radians(angle)
            x = cos(radian) * self.orbitalRadius + self.nodeRotationDegree
            y = sin(radian) * self.orbitalRadius + self.nodeRotationDegree
            new_node = vec(target_pos[0] + x, target_pos[1] + y)
            temp_waypoints.append(new_node)
        
        self._waypoints.extend(temp_waypoints)


    def check_trajectory(self, position) -> None:
        # switch the current trajectory to the next node in the _waypoints, 
        # if the distance between the entity and the node is smaller than the leewayDistance
        if self.desired_distance < self.leewayDistance:
            self.__currentNodeCount = (self.__currentNodeCount + 1) % len(self._waypoints)
        else: 
            # Recalculate the current trajectory if the entity leaves the combined radius of the leewayDistance & orbit
            if self.desired_distance > self.leewayDistance + self.orbitalRadius:
                nearest_node = sorted(self._waypoints, key=lambda node: (node - position).length)[0]
                # set the current targeted node as the one after the closet one
                # --> effective to smooth out the movement
                self.__currentNodeCount = (self._waypoints.index(nearest_node) + 1) % len(self._waypoints)

        self.__currentTrajectory = self._waypoints[self.__currentNodeCount]


    def __str__(self):
        return pformat(vars(self), indent=4, sort_dicts=False)