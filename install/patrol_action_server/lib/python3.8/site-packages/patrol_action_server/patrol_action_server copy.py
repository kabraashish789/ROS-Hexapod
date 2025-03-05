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
from rclpy.qos import QoSProfile
from std_msgs.msg import String, Float64, Float64MultiArray

from rclpy.executors import MultiThreadedExecutor

from custom_interfaces.action import Patrol


class Turtlebot3PatrolServer(Node):

    def __init__(self):
        super().__init__('hexapod_server')

        self.commands_Position_leg = Float64MultiArray()
        self.commands_Position_leg.data = [
            0.0, 0.0, 0.0]
        self.goal = Patrol.Goal()


        qos = QoSProfile(depth=1000)

        # Initialise publishers
        self.position_pub0 = self.create_publisher(Float64MultiArray, '/my_position_controller1/commands', qos)
        self.position_pub1 = self.create_publisher(Float64MultiArray, '/my_position_controller2/commands', qos)
        self.position_pub2 = self.create_publisher(Float64MultiArray, '/my_position_controller3/commands', qos)
        self.position_pub3 = self.create_publisher(Float64MultiArray, '/my_position_controller4/commands', qos)
        self.position_pub4 = self.create_publisher(Float64MultiArray, '/my_position_controller5/commands', qos)
        self.position_pub5 = self.create_publisher(Float64MultiArray, '/my_position_controller6/commands', qos)
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


    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        # Accepts or rejects a client request to begin an action
        self.get_logger().info('Received goal request :)')
        self.goal = goal_request
        self.commands_Position_leg.data[0]=self.goal.revolut1
        self.commands_Position_leg.data[1]=self.goal.revolut2
        self.commands_Position_leg.data[2]=self.goal.revolut3
        
       
        check_if_q_bigT_pi=bool
        check_if_q_bigT_pi=False

        for i in range(3):
            if abs(self.commands_Position_leg.data[i])>=3.14:
                #check_if_q_bigT_pi=True
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
        if self.goal.leg_number==0:
            self.position_pub0.publish(self.commands_Position_leg)
        elif self.goal.leg_number==1:
            self.position_pub1.publish(self.commands_Position_leg)
        elif self.goal.leg_number==2:
            self.position_pub2.publish(self.commands_Position_leg)
        elif self.goal.leg_number==3:
            self.position_pub3.publish(self.commands_Position_leg)
        elif self.goal.leg_number==4:
            self.position_pub4.publish(self.commands_Position_leg)
        elif self.goal.leg_number==5:
            self.position_pub5.publish(self.commands_Position_leg)            

        time.sleep(0.001)
        self.get_logger().info('Joint values: {0}'.format(self.commands_Position_leg))
        # When the action is completed
        goal_handle.succeed()
        result = Patrol.Result()
        result.success = True
        self.get_logger().info('Returning result: {0}'.format(result.success))

        return result

    

def main(args=None):
    rclpy.init(args=args)

    patrol_action_server = Turtlebot3PatrolServer()

    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor()

    rclpy.spin(patrol_action_server,executor=executor)

    


if __name__ == '__main__':
    main()