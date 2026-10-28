# Imports
# test comment for commit
import sys
from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error
from pid import PID_ctrl, P, PD, PI, PID
from rclpy import init, spin, spin_once, shutdown
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry as odom
from localization import localization, rawSensor
from planner import TRAJECTORY_PLANNER, POINT_PLANNER, planner
from controller import controller, trajectoryController
# You may add any other imports you may need/want to use below
# import ...

class decision_maker(Node):
    def __init__(self, publisher_msg, publishing_topic, qos_publisher, goalPoint,
                 rate=10, motion_type=POINT_PLANNER, control_type=P,
                 trajectory_type='parabola', max_linear=0.22, max_angular=2.84):
        super().__init__("decision_maker")
        #TODO Part 4: Create a publisher for the topic responsible for robot's motion
        self.publisher=self.create_publisher(publisher_msg, publishing_topic, qos_publisher)
        publishing_period=1/rate
        self.finished=False
        self.goal_tolerance=0.08
        # Instantiate the controller
        # TODO Part 5: Tune your parameters here
        if motion_type == POINT_PLANNER:
            self.controller=controller(klp=0.2, klv=0.05, kli=0.01, kap=0.8, kav=0.05, kai=0.01,
                                       control_type=control_type, max_linear=max_linear, max_angular=max_angular)
            self.planner=planner(POINT_PLANNER)
        elif motion_type==TRAJECTORY_PLANNER:
            self.controller=trajectoryController(klp=0.2, klv=0.05, kli=0.01, kap=0.8, kav=0.05, kai=0.01,
                                                 control_type=control_type, max_linear=max_linear, max_angular=max_angular)
            self.planner=planner(TRAJECTORY_PLANNER, trajectory_type)
        else:
            print("Error! you don't have this planner", file=sys.stderr)
            raise ValueError('Unknown planner')
        # Instantiate the localization, use rawSensor for now
        self.localizer=localization(rawSensor)
        # Instantiate the planner
        # NOTE: goalPoint is used only for the pointPlanner
        self.goal=self.planner.plan(goalPoint)
        self.create_timer(publishing_period, self.timerCallback)

    def timerCallback(self):
        if self.finished:
            return
        # TODO Part 3: Run the localization node
        spin_once(self.localizer, timeout_sec=0.0)  # Remember that this file is already running the decision_maker node.
        if self.localizer.getPose() is None:
            print("waiting for odom msgs ....")
            return
        vel_msg=Twist()
        # TODO Part 3: Check if you reached the goal
        pose=self.localizer.getPose()
        if type(self.goal) == list:
            reached_goal=calculate_linear_error(pose, self.goal[-1])<self.goal_tolerance
        else:
            reached_goal=calculate_linear_error(pose, self.goal)<self.goal_tolerance
        if reached_goal:
            print("reached goal")
            self.publisher.publish(vel_msg)
            self.controller.PID_angular.logger.save_log()
            self.controller.PID_linear.logger.save_log()
            #TODO Part 3: exit the spin
            self.finished=True
            return
        velocity, yaw_rate = self.controller.vel_request(pose, self.goal, True)
        #TODO Part 4: Publish the velocity to move the robot
        vel_msg.linear.x=float(velocity)
        vel_msg.angular.z=float(yaw_rate)
        self.publisher.publish(vel_msg)

import argparse

def main(args=None):
    if args is None:
        args=argparse.Namespace(motion='point', control='p', trajectory='parabola', robot='tb3', x=-1.0, y=-1.0)
    init()
    # TODO Part 3: You migh need to change the QoS profile based on whether you're using the real robot or in simulation.
    # Remember to define your QoS profile based on the information available in "ros2 topic info /odom --verbose" as explained in Tutorial 3
    odom_qos=QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.VOLATILE,
                        history=HistoryPolicy.KEEP_LAST, depth=10)
    control_type={'p':P, 'pd':PD, 'pi':PI, 'pid':PID}[args.control.lower()]
    max_linear, max_angular=(0.22, 2.84) if args.robot=='tb3' else (0.30, 1.90)
    # TODO Part 4: instantiate the decision_maker with the proper parameters for moving the robot
    if args.motion.lower() == "point":
        DM=decision_maker(Twist, '/cmd_vel', odom_qos, [args.x,args.y], motion_type=POINT_PLANNER,
                          control_type=control_type, max_linear=max_linear, max_angular=max_angular)
    elif args.motion.lower() == "trajectory":
        DM=decision_maker(Twist, '/cmd_vel', odom_qos, [args.x,args.y], motion_type=TRAJECTORY_PLANNER,
                          control_type=control_type, trajectory_type=args.trajectory,
                          max_linear=max_linear, max_angular=max_angular)
    else:
        print("invalid motion type", file=sys.stderr)
        shutdown()
        return
    try:
        while not DM.finished:
            spin_once(DM, timeout_sec=0.1)
        print(f"reached there successfully {DM.localizer.pose}")
    except KeyboardInterrupt:
        print('Stopped by user')
    finally:
        DM.publisher.publish(Twist())
        DM.localizer.destroy_node()
        DM.destroy_node()
        shutdown()

if __name__=="__main__":
    argParser=argparse.ArgumentParser(description="point or trajectory")
    argParser.add_argument("--motion", choices=['point','trajectory'], default="point")
    argParser.add_argument("--control", choices=['p','pd','pi','pid'], default='p')
    argParser.add_argument("--trajectory", choices=['parabola','sigmoid'], default='parabola')
    argParser.add_argument("--robot", choices=['tb3','tb4'], default='tb3')
    argParser.add_argument("--x", type=float, default=-1.0)
    argParser.add_argument("--y", type=float, default=-1.0)
    args = argParser.parse_args()
    main(args)
