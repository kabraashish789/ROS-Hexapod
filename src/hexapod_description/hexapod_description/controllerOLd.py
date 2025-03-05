#!/usr/bin/python3
import rclpy
from rclpy import time
from rclpy.node import Node
from std_msgs.msg import Header
from math import sin, cos ,atan2 ,acos, sqrt,radians,pi 
import math
from geometry_msgs.msg import Quaternion
import time
from array import array
import numpy as np 
import xacro
from sensor_msgs.msg import JointState
from std_msgs.msg import String,Float64, Float64MultiArray,Header
from rclpy.qos import QoSProfile ,ReliabilityPolicy,DurabilityPolicy,HistoryPolicy
from tf2_ros import TransformBroadcaster 
from tf2_msgs.msg._tf_message import TFMessage
from tf2_geometry_msgs import TransformStamped 

from geometry_msgs.msg import Vector3
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

import sys
from rclpy.duration import Duration

class Controller(Node): 
    def __init__(self):
        rclpy.init()
        super().__init__('pub_motor_data')
        self.get_logger().info("Test controller has been started.")
        self.state=JointState()
    
        
        qos_profileSu=QoSProfile(depth=1,
                               history=HistoryPolicy.KEEP_LAST,
                               reliability=ReliabilityPolicy.BEST_EFFORT,
                               durability=DurabilityPolicy.VOLATILE)
        qos_profilePu=QoSProfile(depth=18,
                               history=HistoryPolicy.KEEP_LAST,
                               reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.VOLATILE)
        self.timer_=time.time()
        self.joint_state=JointState()
        self.joint_state.position=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_state.velocity =[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_state.effort = []
        self.joint_state.name = ['Revolute1', 'Revolute2', 'Revolute3', 
                        'Revolute4', 'Revolute5', 'Revolute6', 
                        'Revolute7', 'Revolute8', 'Revolute9', 
                        'Revolute10', 'Revolute11', 'Revolute12', 
                        'Revolute13', 'Revolute14', 'Revolute15', 
                        'Revolute16', 'Revolute17', 'Revolute18'] 
        self.a1=0.6
        self.a2=0.4
        self.a3=0.5
        self.a4=0.35
        #test
        '''deltaX_leg0=np.array([1, -2, 3])
        path_leg0=np.array([1, 2, 3])
        sum=np.array([0, 0, 0])
        subst=np.array([0, 0, 0])
        sum=deltaX_leg0+path_leg0
        subst=deltaX_leg0-path_leg0
        self.get_logger().info(f"sum {(sum)}")
        print('\n')
        self.get_logger().info(f"substr {(subst)}")
        print('\n')
        self.get_logger().info(f"sum1 {(sum[0])}")
        print('\n')
        self.get_logger().info(f"sum2 {(sum[1])}")
        print('\n')
        self.get_logger().info(f"substr1 {(subst[1])}")'''
              
        self.d1=0.6
        self.d2=0.3
        self.d3=0.25
        self.d4=0.7

        self.alpha1=0
        self.alpha2=0
        self.alpha3=0
        self.alpha4=0
        self.counter_for_init_jointsValue=0

        global l1,l2, l3,tibia, coxa, femur, alpha, d1
        self.q1_start=radians(90)  #1.16689
        self.q2_start=radians(-45) #72.6275
        self.q3_start=radians(130)  #1.2594
        alpha=radians(90)
        '''l1=0.03462
        l2=0.081542
        l3=146.604/1000'''
        l1=38/1000
        l2=81/1000
        l3=150/1000
        tibia=l3
        femur=l2
        coxa=l1
        d1=0.0643
        self.xHOme=0.0  #0.1297
        self.yHome= 0.1083
        self.zHome= 0.0922    #-0.221       #-0.0922   #-0.0839
        self.leg0_q1_start=self.q1_start
        self.leg0_q2_start=self.q2_start
        self.leg0_q3_start=self.q3_start

        self.leg1_q1_start=self.q1_start
        self.leg1_q2_start=self.q2_start
        self.leg1_q3_start=self.q3_start

        self.leg2_q1_start=self.q1_start
        self.leg2_q2_start=self.q2_start
        self.leg2_q3_start=self.q3_start

        self.leg3_q1_start=self.q1_start
        self.leg3_q2_start=self.q2_start
        self.leg3_q3_start=self.q3_start

        self.leg4_q1_start=self.q1_start
        self.leg4_q2_start=self.q2_start
        self.leg4_q3_start=self.q3_start

        self.leg5_q1_start=self.q1_start
        self.leg5_q2_start=self.q2_start
        self.leg5_q3_start=self.q3_start
      

        self.position_pub = self.create_publisher( Float64MultiArray, "/my_position_controller/commands",qos_profile=qos_profileSu)
        #self.velocity_pub = self.create_publisher( Float64MultiArray, "/my_velocity_controller/commands",18)
        self.position_pub2 = self.create_publisher( Float64, "/my_position_controller2/commands",qos_profile=qos_profileSu)
       
        self.commands_Position_leg024=Float64MultiArray()
        self.commands_Position_leg024.data=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        self.commands_Position_legStart=Float64MultiArray()
        self.commands_Position_legStart.data=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        self.commands_Position_leg135=Float64MultiArray()
        self.commands_Position_leg135.data=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
      
        self.error=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.position_sub=self.create_subscription(JointState ,"/joint_states", self.my_callback,qos_profileSu)
        self.transformationBroadcaster=TransformBroadcaster(self)
        self.transformStamped=TransformStamped()  
        self.transformStamped.child_frame_id='base_link'
        self.Header=Header()
        self.Header.frame_id='dummy'
        self.transformStamped.header=self.Header
        
        try:
         while rclpy.ok():
            rclpy.spin_once(self)

            lift_z=- 0.03  # +x cm for lifting
            Forw_X=0.03
            For_Y=0.03
            translation=Vector3()
            
            rotation=Quaternion()
          
            N=15 # Number of iterations to achieve the desired goal, play with to change the speed
            
            # leg0,2,4
            # lifting in z-direction
            XGoal_leg0=np.array([self.xHOme, self.yHome, lift_z + self.zHome])
            deltaX_leg0=np.array([0.0, 0.0, 0.0])
            PACt_leg0=np.array([0.0, 0.0, 0.0])
            
            path_leg0=np.array([0.0, 0.0, 0.0])
            qAct_leg0=np.array([self.leg0_q1_start, self.leg0_q2_start, self.leg0_q3_start])

            XGoal_leg2=np.array([self.xHOme, self.yHome, lift_z + self.zHome])
            deltaX_leg2=np.array([0.0, 0.0, 0.0])
            path_leg2=np.array([0.0, 0.0, 0.0])
            PACt_leg2=np.array([0.0, 0.0, 0.0])
            qAct_leg2=np.array([self.leg2_q1_start, self.leg2_q2_start, self.leg2_q3_start])

            XGoal_leg4=np.array([self.xHOme, self.yHome, lift_z + self.zHome])
            deltaX_leg4=np.array([0.0, 0.0, 0.0])
            path_leg4=np.array([0.0, 0.0, 0.0])
            PACt_leg4=np.array([0.0, 0.0, 0.0])
            qAct_leg4=np.array([self.leg4_q1_start, self.leg4_q2_start, self.leg4_q3_start])

            PStart_leg0=np.array([self.xHOme, self.yHome, self.zHome])
            PACt_leg0=PStart_leg0

            PStart_leg2=np.array([self.xHOme, self.yHome, self.zHome])
            PACt_leg2=PStart_leg2

            PStart_leg4=np.array([self.xHOme, self.yHome, self.zHome])
            PACt_leg4=PStart_leg4
            
            i=0
            for i in range(N):
             
              path_leg0=PStart_leg0 + (XGoal_leg0 - PStart_leg0)*(i +1)/(N)
              deltaX_leg0=path_leg0 - PACt_leg0
              
              path_leg2=PStart_leg2 + (XGoal_leg2 - PStart_leg2)*(i +1)/(N)
              deltaX_leg2=path_leg2 - PACt_leg2

              path_leg4=PStart_leg4 + (XGoal_leg4 - PStart_leg4)*(i +1)/(N)
              deltaX_leg4=path_leg4 - PACt_leg4

              dq_leg0=self.inverseKinematics(qAct_leg0[0],qAct_leg0[1],qAct_leg0[2], deltaX_leg0[0], deltaX_leg0[1], deltaX_leg0[2])
              qAct_leg0=qAct_leg0 + dq_leg0
             
              dq_leg2=self.inverseKinematics(qAct_leg2[0],qAct_leg2[1],qAct_leg2[2], deltaX_leg2[0], deltaX_leg2[1], deltaX_leg2[2])
              qAct_leg2=qAct_leg2 + dq_leg2

              dq_leg4=self.inverseKinematics(qAct_leg4[0],qAct_leg4[1],qAct_leg4[2], deltaX_leg4[0], deltaX_leg4[1], deltaX_leg4[2])
              qAct_leg4=qAct_leg4 + dq_leg4
      
             
              self.commands_Position_leg024.data[0]=qAct_leg0[0]
              self.commands_Position_leg024.data[6]=qAct_leg0[1] 
              self.commands_Position_leg024.data[12]=qAct_leg0[2]
              

              self.commands_Position_leg024.data[2]=qAct_leg2[0]
              self.commands_Position_leg024.data[8]=qAct_leg2[1] 
              self.commands_Position_leg024.data[14]=qAct_leg2[2] 

              self.commands_Position_leg024.data[4]=qAct_leg4[0]
              self.commands_Position_leg024.data[10]=qAct_leg4[1] 
              self.commands_Position_leg024.data[16]=qAct_leg4[2]
             
              time.sleep(0.001)
              self.position_pub.publish( self.commands_Position_leg024) 
              self.get_logger().info(f"publishing leg024 lift {(self.commands_Position_leg024)}")
              PACt_leg0=self.forwardKinematics(qAct_leg0[0],qAct_leg0[1], qAct_leg0[2])
              PACt_leg2=self.forwardKinematics(qAct_leg2[0],qAct_leg2[1], qAct_leg2[2])
              PACt_leg4=self.forwardKinematics(qAct_leg4[0],qAct_leg4[1], qAct_leg4[2])
             

               
                
            time.sleep(0.001)   
            # next -movement
            XGoal_leg0=np.array([ PACt_leg0[0] , PACt_leg0[1] + For_Y,PACt_leg0[2]])
            deltaX_leg0=np.array([0.0, 0.0, 0.0])
            path_leg0=np.array([0.0, 0.0, 0.0])

            XGoal_leg2=np.array([PACt_leg2[0] - Forw_X , PACt_leg2[1] , PACt_leg2[2]])
            deltaX_leg2=np.array([0.0, 0.0, 0.0])
            path_leg2=np.array([0.0, 0.0, 0.0])
           

            XGoal_leg4=np.array([PACt_leg4[0] + Forw_X,PACt_leg4[1] , PACt_leg4[2]])
            deltaX_leg4=np.array([0.0, 0.0, 0.0])
            path_leg4=np.array([0.0, 0.0, 0.0])
        
            PStart_leg2=PACt_leg2
            PStart_leg4=PACt_leg4
            PStart_leg0=PACt_leg0
            i=0
            for i in range(N):
              now = self.get_clock().now()
              path_leg0=PStart_leg0 + (XGoal_leg0 - PStart_leg0)*(i +1)/(N)
              deltaX_leg0=path_leg0 - PACt_leg0

              path_leg2=PStart_leg2 + (XGoal_leg2 - PStart_leg2)*(i +1)/(N)
              deltaX_leg2=path_leg2 - PACt_leg2

              path_leg4=PStart_leg4 + (XGoal_leg4 - PStart_leg4)*(i +1)/(N)
              deltaX_leg4=path_leg4 - PACt_leg4

              dq_leg0=self.inverseKinematics(qAct_leg0[0],qAct_leg0[1],qAct_leg0[2], deltaX_leg0[0], deltaX_leg0[1], deltaX_leg0[2])
              qAct_leg0=qAct_leg0+ dq_leg0

              dq_leg2=self.inverseKinematics(qAct_leg2[0],qAct_leg2[1],qAct_leg2[2], deltaX_leg2[0], deltaX_leg2[1], deltaX_leg2[2])
              qAct_leg2=qAct_leg2+ dq_leg2

              dq_leg4=self.inverseKinematics(qAct_leg4[0],qAct_leg4[1],qAct_leg4[2], deltaX_leg4[0], deltaX_leg4[1], deltaX_leg4[2])
              qAct_leg4=qAct_leg4+ dq_leg4

              self.commands_Position_leg024.data[0]=qAct_leg0[0]
              self.commands_Position_leg024.data[6]=qAct_leg0[1] 
              self.commands_Position_leg024.data[12]=qAct_leg0[2] 
              

              self.commands_Position_leg024.data[2]=qAct_leg2[0]
              self.commands_Position_leg024.data[8]=qAct_leg2[1] 
              self.commands_Position_leg024.data[14]=qAct_leg2[2] 

              self.commands_Position_leg024.data[4]=qAct_leg4[0]
              self.commands_Position_leg024.data[10]=qAct_leg4[1] 
              self.commands_Position_leg024.data[16]=qAct_leg4[2] 
              '''translation.x=12.0
              translation.y=0.0
              translation.z=0.0
              rotation.z=0.0
              rotation.w=0.0
              rotation.x=0.0
              rotation.y=0.0
              self.transformStamped.transform.translation=translation
              self.transformStamped.transform.rotation=rotation
              self.transformationBroadcaster.sendTransform(self.transformStamped)'''
              time.sleep(0.001)
              self.position_pub.publish( self.commands_Position_leg024) 
              
              self.get_logger().info(f"publishing leg024 next move {(self.commands_Position_leg024)}")
              PACt_leg0=self.forwardKinematics(qAct_leg0[0],qAct_leg0[1], qAct_leg0[2])
              PACt_leg2=self.forwardKinematics(qAct_leg2[0],qAct_leg2[1], qAct_leg2[2])
              PACt_leg4=self.forwardKinematics(qAct_leg4[0],qAct_leg4[1], qAct_leg4[2])

            time.sleep(0.001)
                
            # leg024 move back to the origine z position
            XGoal_leg0=np.array([self.xHOme, PACt_leg0[1], self.zHome])
            deltaX_leg0=np.array([0.0, 0.0, 0.0])
            path_leg0=np.array([0.0, 0.0, 0.0])

            XGoal_leg2=np.array([PACt_leg2[0],self.yHome, self.zHome])
            deltaX_leg2=np.array([0.0, 0.0, 0.0])
            path_leg2=np.array([0.0, 0.0, 0.0])
           

            XGoal_leg4=np.array([PACt_leg4[0], self.yHome, self.zHome])
            deltaX_leg4=np.array([0.0, 0.0, 0.0])
            path_leg4=np.array([0.0, 0.0, 0.0])
        
            PStart_leg2=PACt_leg2
            PStart_leg4=PACt_leg4
            PStart_leg0=PACt_leg0
            i=0
            for i in range(N):
             
              path_leg0=PStart_leg0 + (XGoal_leg0 - PStart_leg0)*(i +1)/(N)
              deltaX_leg0=path_leg0 - PACt_leg0

              path_leg2=PStart_leg2 + (XGoal_leg2 - PStart_leg2)*(i +1)/(N)
              deltaX_leg2=path_leg2 - PACt_leg2

              path_leg4=PStart_leg4 + (XGoal_leg4 - PStart_leg4)*(i +1)/(N)
              deltaX_leg4=path_leg4 - PACt_leg4

              dq_leg0=self.inverseKinematics(qAct_leg0[0],qAct_leg0[1],qAct_leg0[2], deltaX_leg0[0], deltaX_leg0[1], deltaX_leg0[2])
              qAct_leg0=qAct_leg0+ dq_leg0

              dq_leg2=self.inverseKinematics(qAct_leg2[0],qAct_leg2[1],qAct_leg2[2], deltaX_leg2[0], deltaX_leg2[1], deltaX_leg2[2])
              qAct_leg2=qAct_leg2+ dq_leg2

              dq_leg4=self.inverseKinematics(qAct_leg4[0],qAct_leg4[1],qAct_leg4[2], deltaX_leg4[0], deltaX_leg4[1], deltaX_leg4[2])
              qAct_leg4=qAct_leg4+ dq_leg4
        
            
            
             
              self.commands_Position_leg024.data[0]=qAct_leg0[0]
              self.commands_Position_leg024.data[6]=qAct_leg0[1] 
              self.commands_Position_leg024.data[12]=qAct_leg0[2] 
              

              self.commands_Position_leg024.data[2]=qAct_leg2[0]
              self.commands_Position_leg024.data[8]=qAct_leg2[1] 
              self.commands_Position_leg024.data[14]=qAct_leg2[2] 

              self.commands_Position_leg024.data[4]=qAct_leg4[0]
              self.commands_Position_leg024.data[10]=qAct_leg4[1] 
              self.commands_Position_leg024.data[16]=qAct_leg4[2] 
              
              time.sleep(0.001) 
              self.position_pub.publish( self.commands_Position_leg024) 
              self.get_logger().info(f"publishing leg024 back home {(self.commands_Position_leg024)}")
              PACt_leg0=self.forwardKinematics(qAct_leg0[0],qAct_leg0[1], qAct_leg0[2])
              PACt_leg2=self.forwardKinematics(qAct_leg2[0],qAct_leg2[1], qAct_leg2[2])
              PACt_leg4=self.forwardKinematics(qAct_leg4[0],qAct_leg4[1], qAct_leg4[2])
              
            '''self.leg0_q1_start=qAct_leg0[0]
            self.leg0_q2_start=qAct_leg0[1]
            self.leg0_q3_start=qAct_leg0[2]
            self.leg2_q1_start=qAct_leg2[0]
            self.leg2_q2_start=qAct_leg2[1]
            self.leg2_q3_start=qAct_leg2[2]
            self.leg4_q1_start=qAct_leg4[0]
            self.leg4_q2_start=qAct_leg4[1]
            self.leg4_q3_start=qAct_leg4[2]'''
            
            
            time.sleep(0.001) 
            #leg1,3,5

            # lifting in z-direction
            XGoal_leg1=np.array([self.xHOme, self.yHome, self.zHome +lift_z])
            deltaX_leg1=np.array([0.0, 0.0, 0.0])
            path_leg1=np.array([0.0, 0.0, 0.0])
            PACt_leg1=np.array([0.0, 0.0, 0.0])
            qAct_leg1=np.array([self.leg1_q1_start, self.leg1_q2_start, self.leg1_q3_start])

            XGoal_leg3=np.array([self.xHOme, self.yHome,self.zHome + lift_z])
            deltaX_leg3=np.array([0.0, 0.0, 0.0])
            path_leg3=np.array([0.0, 0.0, 0.0])
            PACt_leg3=np.array([0.0, 0.0, 0.0])
            qAct_leg3=np.array([self.leg3_q1_start, self.leg3_q2_start, self.leg3_q3_start])

            XGoal_leg5=np.array([self.xHOme, self.yHome,self.zHome + lift_z])
            deltaX_leg5=np.array([0.0, 0.0, 0.0])
            path_leg5=np.array([0.0, 0.0, 0.0])
            PACt_leg5=np.array([0.0, 0.0, 0.0])
            qAct_leg5=np.array([self.leg5_q1_start, self.leg5_q2_start, self.leg5_q3_start])

            PStart_leg1=np.array([self.xHOme, self.yHome, self.zHome])
            PACt_leg1=PStart_leg1

            PStart_leg3=np.array([self.xHOme, self.yHome, self.zHome])
            PACt_leg3=PStart_leg3

            PStart_leg5=np.array([self.xHOme, self.yHome, self.zHome])
            PACt_leg5=PStart_leg5
            i=0
            for i in range(N):
              
              path_leg1=PStart_leg1 + (XGoal_leg1 - PStart_leg1)*(i +1)/(N)
              deltaX_leg1=path_leg1 - PACt_leg1
              
              path_leg3=PStart_leg3 + (XGoal_leg3 - PStart_leg3)*(i +1)/(N)
              deltaX_leg3=path_leg3 - PACt_leg3

              path_leg5=PStart_leg5 + (XGoal_leg5 - PStart_leg5)*(i +1)/(N)
              deltaX_leg5=path_leg5 - PACt_leg5

              dq_leg1=self.inverseKinematics(qAct_leg1[0],qAct_leg1[1],qAct_leg1[2], deltaX_leg1[0], deltaX_leg1[1], deltaX_leg1[2])
              qAct_leg1=qAct_leg1+ dq_leg1

              dq_leg3=self.inverseKinematics(qAct_leg3[0],qAct_leg3[1],qAct_leg3[2], deltaX_leg3[0], deltaX_leg3[1], deltaX_leg3[2])
              qAct_leg3=qAct_leg3+ dq_leg3

              dq_leg5=self.inverseKinematics(qAct_leg5[0],qAct_leg5[1],qAct_leg5[2], deltaX_leg5[0], deltaX_leg5[1], deltaX_leg5[2])
              qAct_leg5=qAct_leg5+ dq_leg5
        
              self.commands_Position_leg135.data[1]=qAct_leg1[0]
              self.commands_Position_leg135.data[7]=qAct_leg1[1] 
              self.commands_Position_leg135.data[13]=qAct_leg1[2] 

              self.commands_Position_leg135.data[3]=qAct_leg3[0]
              self.commands_Position_leg135.data[9]=qAct_leg3[1] 
              self.commands_Position_leg135.data[15]=qAct_leg3[2] 

              
              self.commands_Position_leg135.data[5]=qAct_leg5[0]
              self.commands_Position_leg135.data[11]=qAct_leg5[1] 
              self.commands_Position_leg135.data[17]=qAct_leg5[2] 
              time.sleep(0.001)
              self.position_pub.publish( self.commands_Position_leg135) 
              self.get_logger().info(f"publishing leg135 lifting {(self.commands_Position_leg135)}")
              PACt_leg1=self.forwardKinematics(qAct_leg1[0],qAct_leg1[1], qAct_leg1[2])
              PACt_leg3=self.forwardKinematics(qAct_leg3[0],qAct_leg3[1], qAct_leg3[2])
              PACt_leg5=self.forwardKinematics(qAct_leg5[0],qAct_leg5[1], qAct_leg5[2])
              
           
            time.sleep(0.001)  
            # next -movement
            XGoal_leg1=np.array([ PACt_leg1[0] - Forw_X,PACt_leg1[1], PACt_leg1[2]])
            deltaX_leg1=np.array([0.0, 0.0, 0.0])
            path_leg1=np.array([0.0, 0.0, 0.0])

            XGoal_leg3=np.array([PACt_leg3[0] ,PACt_leg3[1] - For_Y,PACt_leg3[2]])
            deltaX_leg3=np.array([0.0, 0.0, 0.0])
            path_leg3=np.array([0.0, 0.0, 0.0])
           

            XGoal_leg5=np.array([PACt_leg5[0] + Forw_X,PACt_leg5[1] , PACt_leg5[2]])
            deltaX_leg5=np.array([0.0, 0.0, 0.0])
            path_leg5=np.array([0.0, 0.0, 0.0])
        
            PStart_leg1=PACt_leg1
            PStart_leg3=PACt_leg3
            PStart_leg5=PACt_leg5
            i=0
            for i in range(N):
             
              path_leg1=PStart_leg1 + (XGoal_leg1 - PStart_leg1)*(i +1)/(N)
              deltaX_leg1=path_leg1 - PACt_leg1
              
              path_leg3=PStart_leg3 + (XGoal_leg3 - PStart_leg3)*(i +1)/(N)
              deltaX_leg3=path_leg3 - PACt_leg3

              path_leg5=PStart_leg5 + (XGoal_leg5 - PStart_leg5)*(i +1)/(N)
              deltaX_leg5=path_leg5 - PACt_leg5

              dq_leg1=self.inverseKinematics(qAct_leg1[0],qAct_leg1[1],qAct_leg1[2], deltaX_leg1[0], deltaX_leg1[1], deltaX_leg1[2])
              qAct_leg1=qAct_leg1+ dq_leg1

              dq_leg3=self.inverseKinematics(qAct_leg3[0],qAct_leg3[1],qAct_leg3[2], deltaX_leg3[0], deltaX_leg3[1], deltaX_leg3[2])
              qAct_leg3=qAct_leg3+ dq_leg3

              dq_leg5=self.inverseKinematics(qAct_leg5[0],qAct_leg5[1],qAct_leg5[2], deltaX_leg5[0], deltaX_leg5[1], deltaX_leg5[2])
              qAct_leg5=qAct_leg5+ dq_leg5
              
              self.commands_Position_leg135.data[1]=qAct_leg1[0]
              self.commands_Position_leg135.data[7]=qAct_leg1[1] 
              self.commands_Position_leg135.data[13]=qAct_leg1[2] 

              self.commands_Position_leg135.data[3]=qAct_leg3[0]
              self.commands_Position_leg135.data[9]=qAct_leg3[1] 
              self.commands_Position_leg135.data[15]=qAct_leg3[2] 


              self.commands_Position_leg135.data[5]=qAct_leg5[0]
              self.commands_Position_leg135.data[11]=qAct_leg5[1] 
              self.commands_Position_leg135.data[17]=qAct_leg5[2] 
           
              time.sleep(0.001)
              self.position_pub.publish( self.commands_Position_leg135) 
              self.get_logger().info(f"publishing leg135 next move {(self.commands_Position_leg135)}")
              PACt_leg1=self.forwardKinematics(qAct_leg1[0],qAct_leg1[1], qAct_leg1[2])
              PACt_leg3=self.forwardKinematics(qAct_leg3[0],qAct_leg3[1], qAct_leg3[2])
              PACt_leg5=self.forwardKinematics(qAct_leg5[0],qAct_leg5[1], qAct_leg5[2])
              
            time.sleep(0.001)
                
            # move back to the origine x,z position
            XGoal_leg1=np.array([ PACt_leg1[0],  self.yHome , self.zHome])
            deltaX_leg1=np.array([0.0, 0.0, 0.0])
            path_leg1=np.array([0.0, 0.0, 0.0])

            XGoal_leg3=np.array([self.xHOme, PACt_leg3[1], self.zHome])
            deltaX_leg3=np.array([0.0, 0.0, 0.0])
            path_leg3=np.array([0.0, 0.0, 0.0])
           

            XGoal_leg5=np.array([ PACt_leg5[0], self.yHome, self.zHome])
            deltaX_leg5=np.array([0.0, 0.0, 0.0])
            path_leg5=np.array([0.0, 0.0, 0.0])
        
            PStart_leg1=PACt_leg1
            PStart_leg3=PACt_leg3
            PStart_leg5=PACt_leg5
            i=0
            for i in range(N):
              
              path_leg1=PStart_leg1 + (XGoal_leg1 - PStart_leg1)*(i +1)/(N)
              deltaX_leg1=path_leg1 - PACt_leg1
              
              path_leg3=PStart_leg3 + (XGoal_leg3 - PStart_leg3)*(i +1)/(N)
              deltaX_leg3=path_leg3 - PACt_leg3

              path_leg5=PStart_leg5 + (XGoal_leg5 - PStart_leg5)*(i +1)/(N)
              deltaX_leg5=path_leg5 - PACt_leg5

              dq_leg1=self.inverseKinematics(qAct_leg1[0],qAct_leg1[1],qAct_leg1[2], deltaX_leg1[0], deltaX_leg1[1], deltaX_leg1[2])
              qAct_leg1=qAct_leg1+ dq_leg1

              dq_leg3=self.inverseKinematics(qAct_leg3[0],qAct_leg3[1],qAct_leg3[2], deltaX_leg3[0], deltaX_leg3[1], deltaX_leg3[2])
              qAct_leg3=qAct_leg3+ dq_leg3

              dq_leg5=self.inverseKinematics(qAct_leg5[0],qAct_leg5[1],qAct_leg5[2], deltaX_leg5[0], deltaX_leg5[1], deltaX_leg5[2])
              qAct_leg5=qAct_leg5+ dq_leg5
        
              self.commands_Position_leg135.data[1]=qAct_leg1[0]
              self.commands_Position_leg135.data[7]=qAct_leg1[1] 
              self.commands_Position_leg135.data[13]=qAct_leg1[2] 

              self.commands_Position_leg135.data[3]=qAct_leg3[0]
              self.commands_Position_leg135.data[9]=qAct_leg3[1] 
              self.commands_Position_leg135.data[15]=qAct_leg3[2] 


              self.commands_Position_leg135.data[5]=qAct_leg5[0]
              self.commands_Position_leg135.data[11]=qAct_leg5[1] 
              self.commands_Position_leg135.data[17]=qAct_leg5[2] 
              time.sleep(0.001)
              self.position_pub.publish( self.commands_Position_leg135) 
              self.get_logger().info(f"publishing  leg135 move back {(self.commands_Position_leg135)}")
              PACt_leg1=self.forwardKinematics(qAct_leg1[0],qAct_leg1[1], qAct_leg1[2])
              PACt_leg3=self.forwardKinematics(qAct_leg3[0],qAct_leg3[1], qAct_leg3[2])
              PACt_leg5=self.forwardKinematics(qAct_leg5[0],qAct_leg5[1], qAct_leg5[2])
              
               
            '''self.leg1_q1_start=qAct_leg1[0]
            self.leg1_q2_start=qAct_leg1[1]
            self.leg1_q3_start=qAct_leg1[2]
            self.leg3_q1_start=qAct_leg3[0]
            self.leg3_q2_start=qAct_leg3[1]
            self.leg3_q3_start=qAct_leg3[2]
            self.leg5_q1_start=qAct_leg5[0]
            self.leg5_q2_start=qAct_leg5[1]
            self.leg5_q3_start=qAct_leg5[2]'''
          
            time.sleep(0.001)
            
        except KeyboardInterrupt:
          pass
       
        

    def my_callback(self, msg: JointState):
         
          if self.counter_for_init_jointsValue==0:
           #xstart=self.forwardKinematics(msg.position[0],msg.position[6],msg.position[12])
           '''self.xHOme=xstart[0]
           self.yHome=xstart[1]
           self.zHome=xstart[2]'''
           
           for i in range(18):
            self.joint_state.position[i]=  msg.position[i]*180/pi

           print("\n")
           self.get_logger().info(f" start angles\n {(msg.position)} ")
           

           '''self.leg0_q1_start=msg.position[0]
           self.leg0_q2_start=msg.position[6]
           self.leg0_q3_start=msg.position[12]
           self.leg2_q1_start=msg.position[0]
           self.leg2_q2_start=msg.position[6]
           self.leg2_q3_start=msg.position[12]
           self.leg4_q1_start=msg.position[0]
           self.leg4_q2_start=msg.position[6]
           self.leg4_q3_start=msg.position[12]
           self.leg1_q1_start=msg.position[0]
           self.leg1_q2_start=msg.position[6]
           self.leg1_q3_start=msg.position[12]
           self.leg3_q1_start=msg.position[0]
           self.leg3_q2_start=msg.position[6]
           self.leg3_q3_start=msg.position[12]
           self.leg5_q1_start=msg.position[0]
           self.leg5_q2_start=msg.position[6]
           self.leg5_q3_start=msg.position[12]'''
            
           '''self.commands_Position_legStart.data[0]=self.q1_start
           self.commands_Position_legStart.data[1]=self.q1_start
           self.commands_Position_legStart.data[2]=self.q1_start
           self.commands_Position_legStart.data[3]=self.q1_start
           self.commands_Position_legStart.data[4]=self.q1_start
           self.commands_Position_legStart.data[5]=self.q1_start
           self.commands_Position_legStart.data[6]=self.q2_start
           self.commands_Position_legStart.data[7]=self.q2_start
           self.commands_Position_legStart.data[8]=self.q2_start
           self.commands_Position_legStart.data[9]=self.q2_start
           self.commands_Position_legStart.data[10]=self.q2_start
           self.commands_Position_legStart.data[11]=self.q2_start
           self.commands_Position_legStart.data[12]=self.q3_start
           self.commands_Position_legStart.data[13]=self.q3_start
           self.commands_Position_legStart.data[14]=self.q3_start
           self.commands_Position_legStart.data[15]=self.q3_start
           self.commands_Position_legStart.data[16]=self.q3_start
           self.commands_Position_legStart.data[17]=self.q3_start
           self.position_pub.publish( self.commands_Position_legStart)'''
          self.counter_for_init_jointsValue+=1
          
          
          
          
          print("\n")
          self.get_logger().info(f"Received start POsition\n {(self.joint_state.position)} ")
          
          print("\n")
    
    def inverseKinematics(self,q1,q2,q3,dx,dy,dz):
         dq=np.array([0.0, 0.0, 0.0])
         det=self.DetOfJacobian(q1,q2,q3)
         self.get_logger().info(f" determinant of the jacobian \n {( det)} ")

         dq[0]=(dy*cos(q1) - dx*sin(q1))/(l1 + l3*cos(q2 + q3) + l2*cos(q2))
         dq[1]=(dz*cos(q2)*sin(q3) + dz*cos(q3)*sin(q2) + dx*cos(q1)*cos(q2)*cos(q3) + dy*cos(q2)*cos(q3)*sin(q1) - dx*cos(q1)*sin(q2)*sin(q3) - dy*sin(q1)*sin(q2)*sin(q3))/(l2*sin(q3))
         dq[2]=-(dz*l2*sin(q2) + dx*l2*cos(q1)*cos(q2) + dy*l2*cos(q2)*sin(q1) + dz*l3*cos(q2)*sin(q3) + dz*l3*cos(q3)*sin(q2) + dx*l3*cos(q1)*cos(q2)*cos(q3) + dy*l3*cos(q2)*cos(q3)*sin(q1) - dx*l3*cos(q1)*sin(q2)*sin(q3) - dy*l3*sin(q1)*sin(q2)*sin(q3))/(l2*l3*sin(q3))
         
         for i in range(3):
           if abs(dq[i])< 0.000000001 :
             dq[i]=0.0
           if dq[i]>3.14:
             dq[i]=3.14 
           if dq[i]<-3.14:
             dq[i]=-3.14    
         self.get_logger().info(f" displacement angles dq \n {( dq)} ")
        
         return dq
    def inverseKinematicsTrigo(self,x,y,z):
        dq=np.array([0.0, 0.0, 0.0])
         
        ll= sqrt(pow(x,2)+pow(y,2))  
        x2=pow(x,2)
        y2=pow(y,2)

        gammaRadians=-atan2(x,y)
        aux= sqrt(x2+y2)-coxa
        HF= sqrt(pow(ll-coxa,2)+pow(z,2))
        alpha1= atan2((ll-coxa),z)
        alpha2= acos(((pow(tibia,2))-pow(femur,2)-pow(HF,2))/(-2*femur*HF))
        alphaRadians= (pi/2)-(alpha1+alpha2)
        betaRadians= acos((pow(tibia,2)+pow(femur,2)-pow(z,2)-pow(aux,2))/(-2*femur*tibia))
        dq[0]=gammaRadians*180/pi
        dq[1]=alphaRadians*180/pi
        dq[2]=betaRadians*180/pi
        self.get_logger().info(f" qs angles \n {( (dq))} ")
        
        return dq
    
    def forwardKinematics(self ,q1,q2,q3):
       xAck=np.array([0.0, 0.0, 0.0])

       xAck[0]=0.5*l2*cos(q1 - 1.0*q2) + 0.5*l2*cos(q1 + q2) + 0.5*l3*cos(q2 - 1.0*q1 + q3) + l1*cos(q1) + 0.5*l3*cos(q1 + q2 + q3)
       xAck[1]=0.5*l2*sin(q1 - 1.0*q2) + 0.5*l2*sin(q1 + q2) - 0.5*l3*sin(q2 - 1.0*q1 + q3) + l1*sin(q1) + 0.5*l3*sin(q1 + q2 + q3)
       xAck[2]= l3*sin(q2 + q3) + l2*sin(q2)
       for i in range(3):
          if abs(xAck[i])< 0.000000001 :
             xAck[i]=0.0
       print("\n")
       self.get_logger().info(f"Actual cartesian position  \n {( xAck)} ")
       return xAck
    
   
    def DetOfJacobian(self,q1,q2,q3):
       det=1.0*l2*l2*l3*sin(q2 + q3)*cos(q1)*cos(q1)*cos(q2)*cos(q2) - 1.0*l2*l3*l3*cos(q2 + q3)*cos(q2 + q3)*cos(q1)*cos(q1)*sin(q2) - 1.0*l2*l3*l3*cos(q2 + q3)*cos(q2 + q3)*sin(q1)*sin(q1)*sin(q2) + 1.0*l2*l2*l3*sin(q2 + q3)*cos(q2)*cos(q2)*sin(q1)*sin(q1) + 1.0*l2*l3*l3*cos(q2 + q3)*sin(q2 + q3)*cos(q2)*sin(q1)*sin(q1) - 1.0*l2*l2*l3*cos(q2 + q3)*cos(q1)*cos(q1)*cos(q2)*sin(q2) - 1.0*l2*l2*l3*cos(q2 + q3)*cos(q2)*sin(q1)*sin(q1)*sin(q2) - 1.0*l1*l2*l3*cos(q2 + q3)*cos(q1)*cos(q1)*sin(q2) + 1.0*l1*l2*l3*sin(q2 + q3)*cos(q1)*cos(q1)*cos(q2) - 1.0*l1*l2*l3*cos(q2 + q3)*sin(q1)*sin(q1)*sin(q2) + 1.0*l1*l2*l3*sin(q2 + q3)*cos(q2)*sin(q1)*sin(q1) + 1.0*l2*l3*l3*cos(q2 + q3)*sin(q2 + q3)*cos(q1)*cos(q1)*cos(q2)
       return det
    
def main():
    myController = Controller()
    
if __name__ == '_main_':
    main()
