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
from math import sin, cos, atan2, acos, sqrt, radians, pi
from geometry_msgs.msg import Twist
import numpy as np
from rclpy.action import ActionClient
from rclpy.action import CancelResponse
from rclpy.action import GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String, Float64, Float64MultiArray
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, LivelinessPolicy
from rclpy.task import Future

from rclpy.executors import MultiThreadedExecutor

from custom_interfaces.action import Patrol


class HexapodClient(Node):

    def __init__(self):
        super().__init__('hexapod_client')
        global l1, l2, l3, tibia, coxa, femur, alpha, d1, length, width, orientation
        length =0.0   #112/1000, 87/1000
        width = length
        orientation = radians(60)
        self.q1_start = radians(0)  # 1.16689
        self.q2_start = radians(-45)  # 72.6275
        self.q3_start = radians(130)  # 1.2594
        alpha = radians(90)
        '''l1=0.03462
        l2=0.081542
        l3=146.604/1000'''
        l1 = 38
        l2 = 81
        l3 = 150
        tibia = l3
        femur = l2
        coxa = l1
        global lift_z, For_Y,Forw_X, For_stance
        lift_z = 20 # + m for lifting
        Forw_X = 0.01
        For_Y = 20
        For_stance=20
        self.xHOme = 0  # 0.10823    #0.1095  #0.1297
        self.yHome = 108
        self.zHome = 92 
        self.leg0_xHome = self.xHOme
        self.leg0_yHome = self.yHome
        self.leg0_zHome = self.zHome
        self.counter_for_init_jointsValue=0

        self.leg0_q1_start = self.q1_start
        self.leg0_q2_start = self.q2_start
        self.leg0_q3_start = self.q3_start

        self.leg1_q1_start = self.q1_start
        self.leg1_q2_start = self.q2_start
        self.leg1_q3_start = self.q3_start

        self.leg2_q1_start = self.q1_start
        self.leg2_q2_start = self.q2_start
        self.leg2_q3_start = self.q3_start

        self.leg3_q1_start = self.q1_start
        self.leg3_q2_start = self.q2_start
        self.leg3_q3_start = self.q3_start

        self.leg4_q1_start = self.q1_start
        self.leg4_q2_start = self.q2_start
        self.leg4_q3_start = self.q3_start

        self.leg5_q1_start = self.q1_start
        self.leg5_q2_start = self.q2_start
        self.leg5_q3_start = self.q3_start
        self.commands_Position_leg024=Float64MultiArray()
        self.commands_Position_leg024.data= [
            0.0, 0.0, 0.0,0.0,0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.commands_Position_leg024_stance=Float64MultiArray()
        self.commands_Position_leg024_stance.data= [
            0.0, 0.0, 0.0,0.0,0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.array_of_q = np.array([self.leg0_q1_start, self.leg1_q1_start, self.leg2_q1_start, self.leg3_q1_start, self.leg4_q1_start, self.leg5_q1_start, self.leg0_q2_start, self.leg1_q2_start, self.leg2_q2_start, self.leg3_q2_start, self.leg4_q2_start, self.leg5_q2_start,
                                    self.leg0_q3_start, self.leg1_q3_start, self.leg2_q3_start, self.leg3_q3_start, self.leg4_q3_start, self.leg5_q3_start])
        
        self.array_of_q_stance = np.array([self.leg0_q1_start, self.leg1_q1_start, self.leg2_q1_start, self.leg3_q1_start, self.leg4_q1_start, self.leg5_q1_start, self.leg0_q2_start, self.leg1_q2_start, self.leg2_q2_start, self.leg3_q2_start, self.leg4_q2_start, self.leg5_q2_start,
                                    self.leg0_q3_start, self.leg1_q3_start, self.leg2_q3_start, self.leg3_q3_start, self.leg4_q3_start, self.leg5_q3_start])
        

        self.array_of_q_start = np.array([self.leg0_q1_start, self.leg1_q1_start, self.leg2_q1_start, self.leg3_q1_start, self.leg4_q1_start, self.leg5_q1_start, self.leg0_q2_start, self.leg1_q2_start, self.leg2_q2_start, self.leg3_q2_start, self.leg4_q2_start, self.leg5_q2_start,
                                    self.leg0_q3_start, self.leg1_q3_start, self.leg2_q3_start, self.leg3_q3_start, self.leg4_q3_start, self.leg5_q3_start])
        
        self.array_of_position=np.array( [
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        for i in range (6):
            self.array_of_position[i]=self.xHOme
            self.array_of_position[i+6]=self.yHome
            self.array_of_position[i+12]=self.zHome
        self.array_of_position_stance=np.array( [
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        
        self.temp_error = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        qos_profileSu = QoSProfile(depth=100000,
                                   history=HistoryPolicy.KEEP_ALL,
                                   reliability=ReliabilityPolicy.BEST_EFFORT,
                                   durability=DurabilityPolicy.VOLATILE,

                                   )
        self.position_sub = self.create_subscription(
            JointState, "/joint_states", self.my_callback, qos_profileSu)

        # Initialise client
        self._action_client = ActionClient(
            self,
            Patrol,
            'patrol'
            )

        self.get_logger().info("my controller  action client has been started...")
        #self.get_logger().info(f" test inverse kinematics [1,-1,0.0]=q?   \n {( self.inverseKinematics(1,-1,0))*180/pi} ")
 
    
    def goal_response_callback(self, future):
        # Get handle for the goal we just sent
        goal_handle = future.result()

        # Return early if goal is rejected
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        # Use goal handle to request the result
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
        
    def get_result_callback(self, future):
        result = future.result().result

        # Log result and shut down ROS 2 cleanly
        self.get_logger().info('Result: {0}'.format(result))
        


    # Waits for server to be available, then sends goal
    def send_goal(self, goal:Float64MultiArray ):
        goal_msg = Patrol.Goal()
        goal_msg.revolut1 = goal.data[0]
        goal_msg.revolut2 = goal.data[1]
        goal_msg.revolut3 = goal.data[2]
        goal_msg.revolut4 = goal.data[3]
        goal_msg.revolut5 = goal.data[4]
        goal_msg.revolut6 = goal.data[5]
        goal_msg.revolut7 = goal.data[6]
        goal_msg.revolut8 = goal.data[7]
        goal_msg.revolut9 = goal.data[8]
        
        goal_msg.leg_number=goal.data[9]
        #t1=self.get_clock().now()
        while not self._action_client.wait_for_server(0.001) :
         self.get_logger().info('I m still waiting for the server...  :(')
       

        #self.get_logger().info('dt: {0}'.format(duration))
        # Returns future to goal handle; client runs feedback_callback after sending the goal
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        

        # Register a callback for when future is complete (i.e. server accepts or rejects goal request)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    # Run when client sends goal
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback))
    
    def my_callback(self, msg: JointState):

        if self.counter_for_init_jointsValue == 0:
            print("\n")
            #self.get_logger().info(f" start joints\n {(msg)} ")
            
            '''for i in range(18):
                self.array_of_q_start[i]=msg.position[i]
            self.array_of_q=self.array_of_q_start
            self.array_of_q_stance=self.array_of_q'''
            #self.array_of_q=self.temp_error
            #self.setStartXYZPosition()
            #self.array_of_position_stance=self.array_of_position
        
            
        else:
            dummy=0
            #self.get_logger().info(f"  joints\n {(msg)} ")
            #self.controllerError(msg)
            
        self.counter_for_init_jointsValue+=1    

    def setStartXYZPosition(self):
       for i in range(6):
          xack=self.forwardKinematics(self.array_of_q[i],self.array_of_q[i+6],self.array_of_q[i+12])  
          self.array_of_position[i]=xack[0]
          self.array_of_position[i+6]=xack[1] 
          self.array_of_position[i+12]=xack[2]
        

    def controllerError(self, msg: JointState):
        self.get_logger().info(f" qact   \n {( qAct_first_leg)} ")
        self.temp_error[legnumber]=qAct_first_leg[0]-msg.position[legnumber ]
        self.temp_error[legnumber+6]=qAct_first_leg[1]-msg.position[legnumber +6]
        self.temp_error[legnumber+12]=qAct_first_leg[2]-msg.position[legnumber +12]
        '''qAct_first_leg[0]=msg.position[legnumber ]
        qAct_first_leg[1]=msg.position[legnumber +6]
        qAct_first_leg[2]=msg.position[legnumber +12]'''
        for i in range(18):
            if abs(self.temp_error[i]) > 0.00000000001:
                continue
            else:
                self.temp_error[i]=0
        #self.get_logger().info(f" controller error  \n {( self.temp_error)} ")

    def limitCheck(self, q1, q2, q3):
        q = np.array([0.0, 0.0, 0.0])
        
        '''if q3>0.75:
             q3=0.75
       elif q3<-0.75:
             q3=-0.75    
       if q2>1:
             q2=1
       elif q2<-1:
             q2=-1 
       if q1>1:
             q1=1
       elif q1<-1:
             q1=-1'''
        q[0] = q1
        q[1] = q2
        q[2] = q3
        
                
        return q
    
    def inverseKinematics(self, xGoal,yGoal,zGoal,legnumber):
           
        '''q = np.array([q1, q2, q3])
        det = self.DetOfJacobian(q1, q2, q3)
        #self.get_logger().info(f" determinant of the jacobian \n {( det)} ")
        dq[0] = (dy*cos(q1) - dx*sin(q1))/(l1 + l3*cos(q2 + q3) + l2*cos(q2))
        dq[1] = -(1.0*(dz*cos(q2)*sin(q3) + dz*cos(q3)*sin(q2) - dx*cos(q1)*cos(q2)*cos(q3) - dy*cos(q2)
                  * cos(q3)*sin(q1) + dx*cos(q1)*sin(q2)*sin(q3) + dy*sin(q1)*sin(q2)*sin(q3)))/(l2*sin(q3))
        dq[2] = (dz*l2*sin(q2) - dx*l2*cos(q1)*cos(q2) - dy*l2*cos(q2)*sin(q1) + dz*l3*cos(q2)*sin(q3) + dz*l3*cos(q3)*sin(q2) - dx*l3*cos(q1)
                 * cos(q2)*cos(q3) - dy*l3*cos(q2)*cos(q3)*sin(q1) + dx*l3*cos(q1)*sin(q2)*sin(q3) + dy*l3*sin(q1)*sin(q2)*sin(q3))/(l2*l3*sin(q3))
        for i in range(3):
            if abs(dq[i]) < 1e-10:
                dq[i] = 0.0'''
       
        
        
        
        x=xGoal+self.array_of_position[legnumber]
        y=yGoal+self.array_of_position[legnumber+6]
        z=zGoal+self.array_of_position[legnumber+12]
        self.array_of_position[legnumber]=x
        self.array_of_position[legnumber+6]=y
        self.array_of_position[legnumber+12]=z
        q=np.array([0.0,0.0,0.0])
        ll= sqrt(pow(x,2)+pow(y,2))    
            
        x2=pow(x,2)
        y2=pow(y,2)

        gammaRadians=-atan2(x,y)
        aux= sqrt(x2+y2)-coxa
        HF= sqrt(pow(ll-coxa,2)+pow(z,2))
        a1= atan2((ll-coxa),z)
        a2= acos(((pow(tibia,2))-pow(femur,2)-pow(HF,2))/(-2*femur*HF))
        alphaRadians= (pi/2)-(a1+a2)
        betaRadians= acos((pow(tibia,2)+pow(femur,2)-pow(z,2)-pow(aux,2))/(-2*femur*tibia))
        q[0]=gammaRadians-radians(0.0)
        q[1]=alphaRadians-radians(-45)
        q[2]=betaRadians-radians(130) 
        return q
    
    def FromRobotBaseToWorld(self, leg_number, xw, yw, zw):
        xyz_in_robot_base = np.array([0.0, 0.0, 0.0])
        '''if leg_number == 0:
            xyz_in_robot_base[0] = -xw - length
            xyz_in_robot_base[1] = -yw
            xyz_in_robot_base[2] = zw
        elif leg_number == 1:
            xyz_in_robot_base[0] = -0.5*xw - 0.866*yw - length
            xyz_in_robot_base[1] = 0.866*xw - 0.5*yw
            xyz_in_robot_base[2] = zw
        elif leg_number == 2:
            xyz_in_robot_base[0] = 0.5*xw - 0.866*yw - length
            xyz_in_robot_base[1] = 0.866*xw + 0.5*yw
            xyz_in_robot_base[2] = zw
        elif leg_number == 3:
            xyz_in_robot_base[0] = xw - length
            xyz_in_robot_base[1] = yw
            xyz_in_robot_base[2] = zw
        elif leg_number == 4:
            xyz_in_robot_base[0] = 0.5*xw + 0.866025403*yw - length
            xyz_in_robot_base[1] = -0.866025403*xw + 0.5*yw
            xyz_in_robot_base[2] = zw
        elif leg_number == 5:
            xyz_in_robot_base[0] = -0.5*xw + 0.866025403*yw - length
            xyz_in_robot_base[1] = -0.866025403*xw - 0.5*yw
            xyz_in_robot_base[2] = zw'''
        
        if leg_number == 0:
            xyz_in_robot_base[0] = -yw
            xyz_in_robot_base[1] = xw
            xyz_in_robot_base[2] = zw
        elif leg_number == 1:
            xyz_in_robot_base[0] = -0.8660*xw - 0.5*yw 
            xyz_in_robot_base[1] = 0.5*xw +0.8660*yw
            xyz_in_robot_base[2] = zw
        elif leg_number == 2:
            xyz_in_robot_base[0] = -0.8660*xw + 0.5*yw 
            xyz_in_robot_base[1] = -0.5*xw - 0.5*yw
            xyz_in_robot_base[2] = zw
        elif leg_number == 3:
            xyz_in_robot_base[0] = yw
            xyz_in_robot_base[1] = -xw
            xyz_in_robot_base[2] = zw
        elif leg_number == 4:
            xyz_in_robot_base[0] = 0.866*xw + 0.5*yw 
            xyz_in_robot_base[1] =- 0.5*xw +0.8660*yw
            xyz_in_robot_base[2] = zw
        elif leg_number == 5:
            xyz_in_robot_base[0] = 0.8660*xw - 0.5*yw 
            xyz_in_robot_base[1] = 0.5*xw + 0.866*yw
            xyz_in_robot_base[2] = zw

        return xyz_in_robot_base

    def FromWorldToRobotBase(self, leg_number, xr, yr, zr):
        xyz_in_world = np.array([0.0, 0.0, 0.0])
        if leg_number == 0:
            xyz_in_world[0] = -xr - length
            xyz_in_world[1] = -yr
            xyz_in_world[2] = zr
        elif leg_number == 1:
            xyz_in_world[0] = -0.5*xr + 0.866*yr - 0.5*length
            xyz_in_world[1] = -0.866*xr - 0.5*yr - 0.866*length
            xyz_in_world[2] = zr
        elif leg_number == 2:

            xyz_in_world[0] = 0.5*xr + 0.866*yr + 0.5*length
            xyz_in_world[1] = -0.866*xr + 0.5*yr - 0.866*length
            xyz_in_world[2] = zr
        elif leg_number == 3:
            xyz_in_world[0] = xr + length
            xyz_in_world[1] = yr
            xyz_in_world[2] = zr
        elif leg_number == 4:
            xyz_in_world[0] = 0.5*xr - 0.866025403*yr + 0.5 * length
            xyz_in_world[1] = 0.866025403*xr + 0.5*yr + 0.866025403 * length
            xyz_in_world[2] = zr
        elif leg_number == 5:
            xyz_in_world[0] = -0.5*xr - 0.866025403*yr - 0.5 * length
            xyz_in_world[1] = 0.866025403*xr - 0.5*yr + 0.866025403 * length
            xyz_in_world[2] = zr

        return xyz_in_world

    def forwardKinematics(self, q1Rad, q2Rad, q3Rad):
        xAck = np.array([0.0, 0.0, 0.0])

        '''xAck[0] = 0.5*l2*cos(q1 - 1.0*q2) + 0.5*l2*cos(q1 + q2) + 0.5 * \
            l3*cos(q2 - 1.0*q1 + q3) + l1*cos(q1) + 0.5*l3*cos(q1 + q2 + q3)
        xAck[1] = 0.5*l2*sin(q1 - 1.0*q2) + 0.5*l2*sin(q1 + q2) - 0.5 * \
            l3*sin(q2 - 1.0*q1 + q3) + l1*sin(q1) + 0.5*l3*sin(q1 + q2 + q3)
        xAck[2] = - 1.0*l3*sin(q2 + q3) - 1.0*l2*sin(q2)
        for i in range(3):
            if abs(xAck[i]) <= 1e-10:
                xAck[i] = 0.0
        self.get_logger().info(f" xack   \n {( xAck)} ")'''
        xFK = (coxa*cos(q1Rad+(pi/2)))+(femur*cos(q2Rad)*cos(q1Rad+(pi/2)))+(tibia*cos(q2Rad)*cos(q3Rad)*cos(q1Rad+(pi/2)))-(tibia*cos(q1Rad+(pi/2))*sin(q2Rad)*sin(q3Rad))
        yFK = (coxa*sin(q1Rad+(pi/2)))+(femur*cos(q2Rad)*sin(q1Rad+(pi/2)))+(tibia*cos(q2Rad)*cos(q3Rad)*sin(q1Rad+(pi/2)))-(tibia*sin(q2Rad)*sin(q3Rad)*sin(q1Rad+(pi/2)))
        zFK = (femur*sin(q2Rad))+(tibia*cos(q2Rad)*sin(q3Rad))+(tibia*cos(q3Rad)*sin(q2Rad))
        xAck[0]=xFK
        xAck[1]=yFK
        xAck[2]=zFK
        self.get_logger().info(f" xack   \n {( xAck)} ")
        return xAck

    def DetOfJacobian(self, q1, q2, q3):
        det = 1.0*l2*l2*l3*sin(q2 + q3)*cos(q1)*cos(q1)*cos(q2)*cos(q2) - 1.0*l2*l3*l3*cos(q2 + q3)*cos(q2 + q3)*cos(q1)*cos(q1)*sin(q2) - 1.0*l2*l3*l3*cos(q2 + q3)*cos(q2 + q3)*sin(q1)*sin(q1)*sin(q2) + 1.0*l2*l2*l3*sin(q2 + q3)*cos(q2)*cos(q2)*sin(q1)*sin(q1) + 1.0*l2*l3*l3*cos(q2 + q3)*sin(q2 + q3)*cos(q2)*sin(q1)*sin(q1) - 1.0*l2*l2*l3*cos(q2 + q3)*cos(q1)*cos(
            q1)*cos(q2)*sin(q2) - 1.0*l2*l2*l3*cos(q2 + q3)*cos(q2)*sin(q1)*sin(q1)*sin(q2) - 1.0*l1*l2*l3*cos(q2 + q3)*cos(q1)*cos(q1)*sin(q2) + 1.0*l1*l2*l3*sin(q2 + q3)*cos(q1)*cos(q1)*cos(q2) - 1.0*l1*l2*l3*cos(q2 + q3)*sin(q1)*sin(q1)*sin(q2) + 1.0*l1*l2*l3*sin(q2 + q3)*cos(q2)*sin(q1)*sin(q1) + 1.0*l2*l3*l3*cos(q2 + q3)*sin(q2 + q3)*cos(q1)*cos(q1)*cos(q2)
        return det

def main(args=None):
    rclpy.init(args=args) 
    action_client = HexapodClient()
    executor = MultiThreadedExecutor()
    global qAct_first_leg, legnumber, GoToSwing, GoToStance,oldX,oldY,oldZ
    GoToSwing=True
    GoToStance=False
    For_Y=30
    For_stance=100
    lift_z=30
    oldX=0
    oldY=0
    oldZ=0
    swing=np.array([0, 2, 4])
    stance=np.array([1,3, 5])
    #legnumber=0
    
    #action_client.setStartXYZPosition()
    #action_client.get_logger().info(f" start joint values \n {( action_client.array_of_q)} ") 
    #action_client.get_logger().info(f" start positions in cartes. \n {( action_client.array_of_position)} ") 
    rclpy.spin_once(action_client)
    
    try:
         while rclpy.ok():
            N =16 # Number of iterations to achieve the desired goal, play with to change the speed
            
            x=0   
            y=0
            z=0
            xs=0
            ys=0
            zs=0

            i=0
           
            phi_start=0
            phi_end=2*pi
            linDis=1.0
            height=-1.0
            dphi=(phi_end-phi_start)/N
            phi=phi_start
            counter=0
            position=np.array([0.0,-0.1,-0.2,-0.3,-0.4,-0.5,-0.6,-0.7,-0.8,-0.9,-1.0,-1.1,-1.2,1.2,1.09,1.1,1.0,0.9,0.8,0.7,0.6,0.5,0.4,0.3,0.2,0.1,0])
            while phi<=phi_end:
              
                
                y=(linDis/(2*pi))*(phi-sin(phi))
                z=(height/(2))*(1-cos(phi))
                if counter>=N/2:
                    z=-z
                action_client.get_logger().info(f" z   \n {( z)} ")
                xyz_in_robot_base_first_leg = action_client.FromRobotBaseToWorld(
                    swing[0], x, y, z )
                xyz_in_robot_base_second_leg = action_client.FromRobotBaseToWorld(
                    swing[1], x, y, z )
                xyz_in_robot_base_third_leg = action_client.FromRobotBaseToWorld(
                    swing[2], x, y, z )
                
                legnumber=swing[0]
                qAct_first_leg=action_client.inverseKinematics(
                        xyz_in_robot_base_first_leg[0], xyz_in_robot_base_first_leg[1], xyz_in_robot_base_first_leg[2],swing[0])
                qAct_second_leg=action_client.inverseKinematics(
                        xyz_in_robot_base_second_leg[0], xyz_in_robot_base_second_leg[1], xyz_in_robot_base_second_leg[2],swing[1])
                qAct_third_leg=action_client.inverseKinematics(
                        xyz_in_robot_base_third_leg[0], xyz_in_robot_base_third_leg[1], xyz_in_robot_base_third_leg[2],swing[2])
                action_client.commands_Position_leg024.data[0] = qAct_first_leg[0]
                action_client.commands_Position_leg024.data[1
                                                       ] = qAct_first_leg[1]
                action_client.commands_Position_leg024.data[2
                                                       ] = qAct_first_leg[2]
                   
                action_client.commands_Position_leg024.data[3] = qAct_second_leg[0]
                action_client.commands_Position_leg024.data[4
                                                       ] = qAct_second_leg[1]
                action_client.commands_Position_leg024.data[5
                                                      ] = qAct_second_leg[2] 
                action_client.commands_Position_leg024.data[6] = qAct_third_leg[0]
                action_client.commands_Position_leg024.data[7
                                                       ] = qAct_third_leg[1]
                action_client.commands_Position_leg024.data[8
                                                       ] = qAct_third_leg[2]
                action_client.commands_Position_leg024.data[9]=legnumber
                action_client.send_goal(action_client.commands_Position_leg024) 
                #time.sleep(0.01)
                
                ys=-y
                xyz_in_robot_base_first_leg = action_client.FromRobotBaseToWorld(
                    stance[0], xs, ys, zs )
                xyz_in_robot_base_second_leg = action_client.FromRobotBaseToWorld(
                    stance[1], xs, ys, zs )
                xyz_in_robot_base_third_leg = action_client.FromRobotBaseToWorld(
                    stance[2], xs, ys, zs )
                
                legnumber=stance[0]
                
                qAct_first_leg=action_client.inverseKinematics(
                        xyz_in_robot_base_first_leg[0], xyz_in_robot_base_first_leg[1], xyz_in_robot_base_first_leg[2],stance[0])
                qAct_second_leg=action_client.inverseKinematics(
                        xyz_in_robot_base_second_leg[0], xyz_in_robot_base_second_leg[1], xyz_in_robot_base_second_leg[2],stance[1])
                qAct_third_leg=action_client.inverseKinematics(
                        xyz_in_robot_base_third_leg[0], xyz_in_robot_base_third_leg[1], xyz_in_robot_base_third_leg[2],stance[2])
                action_client.commands_Position_leg024.data[0] = qAct_first_leg[0]
                action_client.commands_Position_leg024.data[1
                                                       ] = qAct_first_leg[1]
                action_client.commands_Position_leg024.data[2
                                                       ] = qAct_first_leg[2]
                   
                action_client.commands_Position_leg024.data[3] = qAct_second_leg[0]
                action_client.commands_Position_leg024.data[4
                                                       ] = qAct_second_leg[1]
                action_client.commands_Position_leg024.data[5
                                                      ] = qAct_second_leg[2] 
                action_client.commands_Position_leg024.data[6] = qAct_third_leg[0]
                action_client.commands_Position_leg024.data[7
                                                       ] = qAct_third_leg[1]
                action_client.commands_Position_leg024.data[8
                                                       ] = qAct_third_leg[2]
                action_client.commands_Position_leg024.data[9]=legnumber
                action_client.send_goal(action_client.commands_Position_leg024)
               
                phi+=dphi
                counter+=1
                
                time.sleep(0.01)
            #rclpy.shutdown()
            
            # change the sequence
            i=0 
            for i in range(3):
                temp=swing[i]   
                swing[i]=stance[i]
                stance[i]=temp
            
                
    except KeyboardInterrupt:
            pass 



if __name__ == '__main__':
    main()