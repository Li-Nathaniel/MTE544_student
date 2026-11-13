import sys

from utilities import Logger

from rclpy.time import Time

from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error
from rclpy.node import Node
from geometry_msgs.msg import Twist

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry as odom

from sensor_msgs.msg import Imu
from kalman_filter import kalman_filter

from rclpy import init, spin, spin_once

import numpy as np
import message_filters

rawSensors=0
kalmanFilter=1
odom_qos=QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=10)

class localization(Node):
    
    def __init__(self, type, dt, loggerName="robotPose.csv", loggerHeaders=["imu_ax", "imu_ay", "kf_ax", "kf_ay","kf_vx","kf_w","kf_x", "kf_y","stamp"]):

        super().__init__("localizer")

        self.loc_logger=Logger( loggerName , loggerHeaders)
        self.odom_logger=Logger("odomPose.csv", ["odom_x", "odom_y", "orientation", "stamp"])
        self.pose=None
        
        if type==rawSensors:
            self.initRawSensors()
        elif type==kalmanFilter:
            self.initKalmanfilter(dt)
        else:
            print("We don't have this type for localization", sys.stderr)
            return  

    def initRawSensors(self):
        self.create_subscription(odom, "/odom", self.odom_callback, qos_profile=odom_qos)
        
    def initKalmanfilter(self, dt):
        
        # TODO Part 3: Set up the quantities for the EKF (hint: you will need the functions for the states and measurements)

        # Initial state:
        # [x, y, theta, angular_velocity, linear_velocity, acceleration]
        x = np.zeros(6)

        # Process noise covariance (6 states).
        # Controls how much uncertainty is assigned to the motion model.
        Q = 0.01 * np.eye(6)

        # Measurement noise covariance (4 measurements).
        # Controls how much confidence is assigned to IMU and odometry.
        R = 0.5 * np.eye(4)

        # Initial state covariance.
        # Initially assume moderate uncertainty in all six states.
        P = 0.1 * np.eye(6)

        # Initialize the Kalman filter.
        self.kf = kalman_filter(P, Q, R, x, dt)
        
        # TODO Part 3: Use the odometry and IMU data for the EKF
        # Subscribe to odometry measurements.
        self.odom_sub = message_filters.Subscriber(
            self,
            odom,
            "/odom",
            qos_profile=odom_qos
        )

        # Subscribe to IMU measurements.
        self.imu_sub = message_filters.Subscriber(
            self,
            Imu,
            "/imu",
            qos_profile=odom_qos
        )

        # Synchronize odometry and IMU messages.
        # Messages must have timestamps within 0.1 seconds.
        time_syncher = message_filters.ApproximateTimeSynchronizer(
            [self.odom_sub, self.imu_sub],
            queue_size=10,
            slop=0.1
        )

        # Call fusion_callback whenever synchronized messages arrive.
        time_syncher.registerCallback(self.fusion_callback)
    
    def fusion_callback(self, odom_msg: odom, imu_msg: Imu):
        
        # TODO Part 3: Use the EKF to perform state estimation
        # Take the measurements
        # your measurements are the linear velocity and angular velocity from odom msg
        # and linear acceleration in x and y from the imu msg
        # the kalman filter should do a proper integration to provide x,y and filter ax,ay

        # Extract measurements from odometry.
        linear_velocity = odom_msg.twist.twist.linear.x
        angular_velocity = odom_msg.twist.twist.angular.z

        # Extract acceleration measurements from the IMU.
        imu_ax = imu_msg.linear_acceleration.x
        imu_ay = imu_msg.linear_acceleration.y

        # Construct the measurement vector:
        # z = [v, w, ax, ay]
        z = np.array([
            linear_velocity,
            angular_velocity,
            imu_ax,
            imu_ay
        ])

        # Perform EKF prediction.
        self.kf.predict()

        # Perform EKF measurement update.
        self.kf.update(z)

        # Get the estimated states.
        xhat = self.kf.get_states()

        # Update the pose estimate returned by getPose().
        self.pose = np.array([
            xhat[0],
            xhat[1],
            xhat[2],
            odom_msg.header.stamp
        ], dtype=object)

        # TODO Part 4: log your data
        # Calculate the estimated acceleration measurements.
        kf_acceleration = self.kf.measurement_model()

        # Extract the timestamp in nanoseconds.
        stamp = Time.from_msg(odom_msg.header.stamp).nanoseconds

        # Log sensor measurements and EKF estimates.
        self.loc_logger.log_values([
            imu_ax,
            imu_ay,
            kf_acceleration[2],
            kf_acceleration[3],
            xhat[4],
            xhat[3],
            xhat[0],
            xhat[1],
            stamp
        ])

        self.odom_logger.log_values([
            euler_from_quaternion(odom_msg.pose.pose.orientation),
            odom_msg.pose.pose.position.x,
            odom_msg.pose.pose.position.y,
            stamp
        ])
      
    def odom_callback(self, pose_msg):
        
        self.pose=[ pose_msg.pose.pose.position.x,
                    pose_msg.pose.pose.position.y,
                    euler_from_quaternion(pose_msg.pose.pose.orientation),
                    pose_msg.header.stamp]

    



        

    # Return the estimated pose
    def getPose(self):
        return self.pose


if __name__=="__main__":
    
    init()
    
    LOCALIZER=localization()
    
    spin(LOCALIZER)
