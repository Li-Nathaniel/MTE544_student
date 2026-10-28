from math import exp
# Type of planner
POINT_PLANNER=0; TRAJECTORY_PLANNER=1

class planner:
    def __init__(self, type_, trajectory_type="parabola"):
        self.type=type_
        self.trajectory_type=trajectory_type

    def plan(self, goalPoint=[-1.0, -1.0]):
        if self.type==POINT_PLANNER:
            return self.point_planner(goalPoint)
        elif self.type==TRAJECTORY_PLANNER:
            return self.trajectory_planner()

    def point_planner(self, goalPoint):
        x = goalPoint[0]
        y = goalPoint[1]
        return x, y

    # TODO Part 6: Implement the trajectories here
    def trajectory_planner(self):
        # the return should be a list of trajectory points: [ [x1,y1], ..., [xn,yn]]
        # return
        if self.trajectory_type == 'parabola':
            return [[i*0.025, (i*0.025)**2] for i in range(61)]
        if self.trajectory_type == 'sigmoid':
            return [[i*0.025, 2/(1+exp(-2*i*0.025))-1] for i in range(101)]
        raise ValueError('Unknown trajectory: '+self.trajectory_type)
