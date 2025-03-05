#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Ryan Shim, Gilbert

import math
import time

import rclpy

from geometry_msgs.msg import Twist

from rclpy.action import ActionServer
from rclpy.action import CancelResponse
from rclpy.action import GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy import publisher 
from rclpy.qos import QoSProfile
from std_msgs.msg import String, Float64, Float64MultiArray
import numpy as np
from rclpy.executors import MultiThreadedExecutor

from custom_interfaces.action import Patrol


class Turtlebot3PatrolServer(Node):

    def __init__(self):
        super().__init__('hexapod_server')

        self.goal = Patrol.Goal()
        self.publisher=[publisher]*18
        self.commands_Position_leg=[Float64MultiArray]*18
        self.commands_Position_leg_temp=Float64MultiArray()
        self.commands_Position_leg_temp.data=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        self.qos = QoSProfile(depth=1000)
        for i in range(18):
            self.createPublisher(i)
            self.createCommandPosition(i)
 
        # Initialise servers
        self._action_server = ActionServer(
            self,
            Patrol,
            'patrol',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)
       
        self.get_logger().info("my controller  action server has been initialised.")

    
    def createPublisher(self, i :int):
        self.publisher[i]=self.create_publisher(Float64MultiArray, '/my_position_controller' + str(i+1)+ '/commands', self.qos)
        
    def  createCommandPosition(self, i:int):
        self.commands_Position_leg[i]=Float64MultiArray()
        self.commands_Position_leg[i].data=[0.0]

    def copy_the_joint (self,leg_number:int) :
                self.commands_Position_leg[leg_number].data[0]=self.commands_Position_leg_temp.data[0]
                self.commands_Position_leg[leg_number+2].data[0]=self.commands_Position_leg_temp.data[3]
                self.commands_Position_leg[leg_number+4].data[0]=self.commands_Position_leg_temp.data[6]
                
                self.commands_Position_leg[leg_number+6].data[0]=self.commands_Position_leg_temp.data[1]
                self.commands_Position_leg[leg_number+8].data[0]=self.commands_Position_leg_temp.data[4]
                self.commands_Position_leg[leg_number+10].data[0]=self.commands_Position_leg_temp.data[7]
                
                self.commands_Position_leg[leg_number+12].data[0]=self.commands_Position_leg_temp.data[2]
                self.commands_Position_leg[leg_number+14].data[0]=self.commands_Position_leg_temp.data[5]
                self.commands_Position_leg[leg_number+16].data[0]=self.commands_Position_leg_temp.data[8]
    
    def publish_joint_value(self, leg_number:int):
                self.publisher[leg_number].publish(self.commands_Position_leg[leg_number]) 
                self.publisher[leg_number+2].publish(self.commands_Position_leg[leg_number+2]) 
                self.publisher[leg_number+4].publish(self.commands_Position_leg[leg_number+4]) 

                self.publisher[leg_number+6].publish(self.commands_Position_leg[leg_number+6]) 
                self.publisher[leg_number+8].publish(self.commands_Position_leg[leg_number+8]) 
                self.publisher[leg_number+10].publish(self.commands_Position_leg[leg_number+10]) 

                self.publisher[leg_number+12].publish(self.commands_Position_leg[leg_number+12]) 
                self.publisher[leg_number+14].publish(self.commands_Position_leg[leg_number+14]) 
                self.publisher[leg_number+16].publish(self.commands_Position_leg[leg_number+16])

                '''self.get_logger().info('Joint values for second leg 1: {0}'.format((self.commands_Position_leg[leg_number+2]) )) 
                self.get_logger().info('Joint values for second leg 2: {0}'.format((self.commands_Position_leg[leg_number+8])))
                self.get_logger().info('Joint values for second leg 3: {0}'.format((self.commands_Position_leg[leg_number+14])))''' 
                
    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        # Accepts or rejects a client request to begin an action
        self.get_logger().info('Received goal request :)')
        self.goal = goal_request
        self.commands_Position_leg_temp.data[0]=self.goal.revolut1
        self.commands_Position_leg_temp.data[1]=self.goal.revolut2
        self.commands_Position_leg_temp.data[2]=self.goal.revolut3
        self.commands_Position_leg_temp.data[3]=self.goal.revolut4
        self.commands_Position_leg_temp.data[4]=self.goal.revolut5
        self.commands_Position_leg_temp.data[5]=self.goal.revolut6
        self.commands_Position_leg_temp.data[6]=self.goal.revolut7
        self.commands_Position_leg_temp.data[7]=self.goal.revolut8
        self.commands_Position_leg_temp.data[8]=self.goal.revolut9
       
       
        check_if_q_bigT_pi=bool
        check_if_q_bigT_pi=False

        for i in range(3):

            if abs(self.commands_Position_leg_temp.data[i])>=3.14:
                check_if_q_bigT_pi=True
                continue
        if check_if_q_bigT_pi:
            goal_response=GoalResponse.REJECT
        else:
          goal_response=GoalResponse.ACCEPT              
        
        return goal_response

    def cancel_callback(self, goal_handle):
        # Accepts or rejects a client request to cancel an action
        self.get_logger().info('Received cancel request :(')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):

        self.get_logger().info('Executing goal...')
      
        leg_number=int(self.goal.leg_number)
        self.copy_the_joint(leg_number)
        self.publish_joint_value(leg_number)
        
        #self.get_logger().info('Joint values: {0}'.format(self.commands_Position_leg))
        # When the action is completed
        goal_handle.succeed()
        result = Patrol.Result()
        result.success = True
        self.get_logger().info('Returning result: {0}'.format(result.success))
        #rclpy.shutdown()
        return result

    

def main(args=None):
    rclpy.init(args=args)

    patrol_action_server = Turtlebot3PatrolServer()

    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor()

    rclpy.spin(patrol_action_server,executor=executor)

    


if __name__ == '__main__':
    main()