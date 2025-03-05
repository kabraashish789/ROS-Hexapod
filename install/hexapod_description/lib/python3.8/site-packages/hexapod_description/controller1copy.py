#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from math import sin, cos ,pi 
import math
from geometry_msgs.msg import Quaternion
import time
from array import array
import numpy as np 
import xacro
from sensor_msgs.msg import JointState
from std_msgs.msg import String,Float64, Float64MultiArray
from rclpy.qos import QoSProfile ,ReliabilityPolicy,DurabilityPolicy,HistoryPolicy
from tf2_ros import TransformBroadcaster ,TransformStamped
from tf2_msgs.msg._tf_message import TFMessage
from tf2_geometry_msgs import TransformStamped

from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

import sys
from rclpy.duration import Duration
#self._action_client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
class ControllerClient(Node):
    def __init__(self):
        rclpy.init()
        super().__init__('motor_data')
        self.get_logger().info("My Test controller has been started.")
        self._action_client = ActionClient(self, Float64MultiArray, "/my_position_controller/commands")
        global angle
        angle=0.99
        self.get_logger().info('called1:')
        try:
         self.get_logger().info('called11:')
         while rclpy.ok():
          rclpy.spin_once(self)
          self.get_logger().info('called2:')
          #rclpy.spin_once(self)
          self.get_logger().info('called22:')
          goal_msg = FollowJointTrajectory.Goal()

        # Fill in data for trajectory
          joint_names = ['Revolute_1', 'Revolute_2', 'Revolute_3', 
                        'Revolute_4', 'Revolute_5', 'Revolute_6', 
                        'Revolute_7', 'Revolute_8', 'Revolute_9', 
                        'Revolute_10', 'Revolute_11', 'Revolute_12', 
                        'Revolute_13', 'Revolute_14', 'Revolute_15', 
                        'Revolute_16', 'Revolute_17', 'Revolute_18'] 

          points = []
          point1 = JointTrajectoryPoint()
          point1.positions = [0.0, 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 ]

          point2 = JointTrajectoryPoint()
          point2.time_from_start = Duration(seconds=1, nanoseconds=0).to_msg()
          point2.positions = [angle, angle , angle , angle , angle , angle , angle , angle , angle , angle , angle , angle , angle  , angle , angle , angle , angle , angle]

          points.append(point1)
          points.append(point2)
          self.get_logger().info('called23:')
          goal_msg.goal_time_tolerance = Duration(seconds=1, nanoseconds=0).to_msg()
          goal_msg.trajectory.joint_names = joint_names
          goal_msg.trajectory.points = points
          
          test=self._action_client.wait_for_server(timeout_sec=1)
          self.get_logger().info(
                        f"the server action is {(test)}")
                   
          if test :
           self.get_logger().info('Received server:')
           _send_goal_future = self._action_client.send_goal(goal_msg, feedback_callback=self.feedback_callback)

           _send_goal_future.add_done_callback(self.goal_response_callback)
          elif test==0:
           rclpy.shutdown()
          angle+=0.01 

        except KeyboardInterrupt:
          pass
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: '+str(result))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback:'+str(feedback))

    

def main():

    action_client = ControllerClient()

   

    
    
     

if __name__ == '__main__':
    main()