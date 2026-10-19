# Imports
import rclpy


from rclpy.node import Node


from utilities import Logger, euler_from_quaternion
from rclpy.qos import QoSProfile


# TODO Part 3: Import message types needed:
   # For sending velocity commands to the robot: Twist
   # For the sensors: Imu, LaserScan, and Odometry
# Check the online documentation to fill in the lines below
from geometry_msgs.msg import Twist # message carried by cmd_vel topic
from sensor_msgs.msg import Imu # message carried by imu topic
from sensor_msgs.msg import LaserScan # message carried by scan topic
from nav_msgs.msg import Odometry


from rclpy.time import Time


# You may add any other imports you may need/want to use below
from rclpy.qos import ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
# constants & list used for each maneuver
CIRCLE=0; SPIRAL=1; ACC_LINE=2
motion_types=['circle', 'spiral', 'line']
# actual node creation instance
class motion_executioner(Node):
  
   def __init__(self, motion_type=0):
      
	# creating internal class-related parameters & variables
       super().__init__("motion_types")
      
       self.type=motion_type
      
       self.radius_=0.0
       self.linear_velocity=0.0
      
       self.successful_init=False
       self.imu_initialized=False
       self.odom_initialized=False
       self.laser_initialized=False
      
       # TODO Part 3: Create a publisher to send velocity commands by setting the proper parameters in (...)
       self.vel_publisher=self.create_publisher(Twist, 'cmd_vel', 10)
              
       # automated loggers with exporting info/declarations
       self.imu_logger=Logger('imu_content_'+str(motion_types[motion_type])+'.csv', headers=["acc_x", "acc_y", "angular_z", "stamp"])
       self.odom_logger=Logger('odom_content_'+str(motion_types[motion_type])+'.csv', headers=["x","y","th", "stamp"])
       self.laser_logger=Logger('laser_content_'+str(motion_types[motion_type])+'.csv', headers=["ranges", "angle_increment", "stamp"])
      
       # TODO Part 3: Create the QoS profile by setting the proper parameters in (...)
       qos=QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.VOLATILE,
                      history=HistoryPolicy.KEEP_LAST,
                      depth=10)


       # TODO Part 5: Create below the subscription to the topics corresponding to the respective sensors
       # IMU subscription
      
       self.vel_publisher=self.create_publisher(Twist, "/cmd_vel", qos)
       self.imu_subscriber=self.create_subscription(Imu, "/imu", self.imu_callback,qos)
     
      
       # ENCODER subscription


       self.laser_subscriber=self.create_subscription(LaserScan, "/scan", self.laser_callback, qos)
      
       # LaserScan subscription
      
       self.odom_subscriber=self.create_subscription(Odometry, "/odom", self.odom_callback, qos)
      
       self.create_timer(0.1, self.timer_callback)




   # TODO Part 5: Callback functions: complete the callback functions of the three sensors to log the proper data.
   # To also log the time you need to use the rclpy Time class, each ros msg will come with a header, and then
   # inside the header you have a stamp that has the time in seconds and nanoseconds, you should log it in nanoseconds as
   # such: Time.from_msg(imu_msg.header.stamp).nanoseconds
   # You can save the needed fields into a list, and pass the list to the log_values function in utilities.py


   def imu_callback(self, imu_msg: Imu):
       # extract IMU measurements
       acc_x = imu_msg.linear_acceleration.x
       acc_y = imu_msg.linear_acceleration.y
       angular_z = imu_msg.angular_velocity.z


       # convert ROS timestamps to nanoseconds
       stamp = Time.from_msg(
           imu_msg.header.stamp
       ).nanoseconds
	# store values in a usable list
       values = [
           acc_x,
           acc_y,
           angular_z,
           stamp
       ]


       self.imu_logger.log_values(values)


       # indicate IMU data has been received
       self.imu_initialized = True
      
   def odom_callback(self, odom_msg: Odometry):
  	
# get robot position and orientation
       x = odom_msg.pose.pose.position.x
       y = odom_msg.pose.pose.position.y


       orientation = odom_msg.pose.pose.orientation
	# store values in a usable list
       quaternion = [
           orientation.x,
           orientation.y,
           orientation.z,
           orientation.w
       ]


       # Convert quaternion directly to yaw (theta)
       th = euler_from_quaternion(quaternion)


       stamp = Time.from_msg(
           odom_msg.header.stamp
       ).nanoseconds
	# store values in a usable list
       values = [
           x,
           y,
           th,
           stamp
       ]


       self.odom_logger.log_values(values)


       self.odom_initialized = True


   def laser_callback(self, laser_msg: LaserScan):
      
       # laser range measurements
       ranges = list(laser_msg.ranges)


       # angular distance between measurements
       angle_increment = laser_msg.angle_increment


       # timestamp
       stamp = Time.from_msg(
           laser_msg.header.stamp
       ).nanoseconds
	# store values in a usable list
       values = [
           ranges,
           angle_increment,
           stamp
       ]


       self.laser_logger.log_values(values)


       # indicate that laser data has been received
       self.laser_initialized = True


   def timer_callback(self):
      
       if self.odom_initialized and self.laser_initialized and self.imu_initialized:
           self.successful_init=True
          
       if not self.successful_init:
           return
      
       cmd_vel_msg=Twist()
      
       if self.type==CIRCLE:
           cmd_vel_msg=self.make_circular_twist()
      
       elif self.type==SPIRAL:
           cmd_vel_msg=self.make_spiral_twist()
                      
       elif self.type==ACC_LINE:
           cmd_vel_msg=self.make_acc_line_twist()
          
       else:
           print("type not set successfully, 0: CIRCLE 1: SPIRAL and 2: ACCELERATED LINE")
           raise SystemExit


       self.vel_publisher.publish(cmd_vel_msg)
      
  
   # TODO Part 4: Motion functions: complete the functions to generate the proper messages corresponding to the desired motions of the robot


   def make_circular_twist(self):
       msg=Twist()
       msg.linear.x = 0.3 # m/s
       msg.angular.z = 1.0 # rad/s
       return msg


   def make_spiral_twist(self):
       msg=Twist()
       self.radius_ += 0.005 # m
       msg.angular.z = 0.5 # rad/s
       # v = r * w
       msg.linear.x = self.radius_ * msg.angular.z # m/s
       return msg
  
   def make_acc_line_twist(self):
       msg=Twist()
       self.linear_velocity += 0.05 # m/s
       msg.linear.x = self.linear_velocity # m/s
       msg.angular.z = 0.0 # rad/s
       return msg


import argparse


if __name__=="__main__":
  


   argParser=argparse.ArgumentParser(description="input the motion type")




   argParser.add_argument("--motion", type=str, default="circle")






   rclpy.init()


   args = argParser.parse_args()


   if args.motion.lower() == "circle":


       ME=motion_executioner(motion_type=CIRCLE)
   elif args.motion.lower() == "line":
       ME=motion_executioner(motion_type=ACC_LINE)


   elif args.motion.lower() =="spiral":
       ME=motion_executioner(motion_type=SPIRAL)


   else:
       print(f"we don't have {arg.motion.lower()} motion type")




  
   try:
       rclpy.spin(ME)
   except KeyboardInterrupt:
       print("Exiting")
