#!/usr/bin/python3
import rclpy
from rclpy import time, duration, clock
from rclpy.node import Node
from std_msgs.msg import Header
from math import sin, cos, atan2, acos, sqrt, radians, pi
import time
from geometry_msgs.msg import Quaternion
from array import array
import numpy as np
import xacro
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float64, Float64MultiArray, Header
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, LivelinessPolicy
from tf2_ros import TransformBroadcaster
from tf2_msgs.msg._tf_message import TFMessage
from tf2_geometry_msgs import TransformStamped
from geometry_msgs.msg import Vector3
from rclpy.action import ActionClient
from rclpy.waitable import Waitable
from rclpy.callback_groups import CallbackGroup
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.qos_event import QoSEventHandler
from rclpy.qos_event import PublisherEventCallbacks
import sys
from rclpy.duration import Duration
from enum import IntEnum
from rclpy.handle import Handle
from action_msgs import msg
from rclpy.impl.implementation_singleton import rclpy_pycapsule_implementation

from rclpy.impl.implementation_singleton import rclpy_handle_implementation as _rclpy_handle


class Controller(Node):
    def __init__(self):
        rclpy.init()
        super().__init__('pub_motor_data')
        self.get_logger().info("Test One Leg controller has been started.")
        self.state = JointState()

        qos_profileSu = QoSProfile(depth=100000,
                                   history=HistoryPolicy.KEEP_ALL,
                                   reliability=ReliabilityPolicy.BEST_EFFORT,
                                   durability=DurabilityPolicy.VOLATILE,

                                   )

        qos_profilePu = QoSProfile(depth=100000,
                                   history=HistoryPolicy.KEEP_ALL,
                                   reliability=ReliabilityPolicy.RELIABLE,
                                   durability=DurabilityPolicy.SYSTEM_DEFAULT,

                                   )
        qos_profilePu.liveliness = LivelinessPolicy.RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC

        self.joint_state = JointState()
        self.joint_state.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_state.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_state.effort = []
        self.joint_state.name = ['Revolute1', 'Revolute2', 'Revolute3',
                                 'Revolute4', 'Revolute5', 'Revolute6',
                                 'Revolute7', 'Revolute8', 'Revolute9',
                                 'Revolute10', 'Revolute11', 'Revolute12',
                                 'Revolute13', 'Revolute14', 'Revolute15',
                                 'Revolute16', 'Revolute17', 'Revolute18']
        self.a1 = 0.6
        self.a2 = 0.4
        self.a3 = 0.5
        self.a4 = 0.35

        self.d1 = 0.6
        self.d2 = 0.3
        self.d3 = 0.25
        self.d4 = 0.7

        self.alpha1 = 0
        self.alpha2 = 0
        self.alpha3 = 0
        self.alpha4 = 0
        self.counter_for_init_jointsValue = 0

        global l1, l2, l3, tibia, coxa, femur, alpha, d1, length, width, orientation
        length = 112/1000
        width = length
        orientation = radians(60)
        self.q1_start = radians(0)  # 1.16689
        self.q2_start = radians(-45)  # 72.6275
        self.q3_start = radians(130)  # 1.2594
        alpha = radians(90)
        '''l1=0.03462
        l2=0.081542
        l3=146.604/1000'''
        l1 = 38/1000
        l2 = 81/1000
        l3 = 150/1000
        tibia = l3
        femur = l2
        coxa = l1
        d1 = 0.0643
        self.xHOme = 0.1083  # 0.10823    #0.1095  #0.1297
        self.yHome = 0.0
        self.zHome = -0.221  # -0.0883 #-0.221      #-0.0922   #-0.0839
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
        self.array_of_q = np.array([self.leg0_q1_start, self.leg1_q1_start, self.leg2_q1_start, self.leg3_q1_start, self.leg4_q1_start, self.leg5_q1_start, self.leg0_q2_start, self.leg1_q2_start, self.leg2_q2_start, self.leg3_q2_start, self.leg4_q2_start, self.leg5_q2_start,
                                    self.leg0_q3_start, self.leg1_q3_start, self.leg2_q3_start, self.leg3_q3_start, self.leg4_q3_start, self.leg5_q3_start])
        self.array_of_position=np.array( [
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        self.leg0_xHome = self.xHOme
        self.leg0_yHome = self.yHome
        self.leg0_zHome = self.zHome

        self.leg1_xHome = self.xHOme
        self.leg1_yHome = self.yHome
        self.leg1_zHome = self.zHome

        self.leg2_xHome = self.xHOme
        self.leg2_yHome = self.yHome
        self.leg2_zHome = self.zHome

        self.leg3_xHome = self.xHOme
        self.leg3_yHome = self.yHome
        self.leg3_zHome = self.zHome

        self.leg4_xHome = self.xHOme
        self.leg4_yHome = self.yHome
        self.leg4_zHome = self.zHome

        self.leg5_xHome = self.xHOme
        self.leg5_yHome = self.yHome
        self.leg5_zHome = self.zHome

        self.event_call_back = PublisherEventCallbacks()
        
        self.position_pub = self.create_publisher(Float64MultiArray, "/my_position_controller/commands",
                                                  qos_profile=qos_profilePu, callback_group=self.default_callback_group, event_callbacks=self.event_call_back)
        self.position_pub.qos_profile._deadline = 0.001
        self.position_pub.qos_profile._lifespan = 0.001
        

        # self.position_pub.event_handlers=QoSEventHandler(self.default_callback_group,callback=self.pu_call_back,event_type=enum,parent_handle=mycap)

        self.velocity_pub = self.create_publisher(
            Float64MultiArray, "/my_velocity_controller/commands", 18)
        # self.position_pub2 = self.create_publisher( Float64, "/my_position_controller2/commands",qos_profile=qos_profileSu)
        self.default_callback_group.add_entity(self.position_pub)
        
        self.get_logger().info(
                        f"has been added? {(self.default_callback_group.has_entity(self.position_pub))}")
        test=self.default_callback_group.can_execute(self.position_pub)
        
        self.get_logger().info(
                        f" {(test)}")
        self.commands_Position_leg024 = Float64MultiArray()
        self.commands_Position_leg024.data = [
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.commands_Velocity_leg024 = Float64MultiArray()
        self.commands_Velocity_leg024.data = [
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.commands_Position_legStart = Float64MultiArray()
        self.commands_Position_legStart.data = [
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.commands_Position_leg135 = Float64MultiArray()
        self.commands_Position_leg135.data = [
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.error = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.position_sub = self.create_subscription(
            JointState, "/joint_states", self.my_callback, qos_profileSu)
        self.transformationBroadcaster = TransformBroadcaster(self)
        self.transformStamped = TransformStamped()
        self.transformStamped.child_frame_id = 'base_link'
        self.Header = Header()
        self.Header.frame_id = 'dummy'
        self.transformStamped.header = self.Header
        lift_z = 0.02  # + m for lifting
        Forw_X = 0.01
        For_Y = 0.02
        iteration = 0
        global legnumber
        self.t = 1
        

        try:
            while rclpy.ok():
               rclpy.spin_once(self)

               if (For_Y >= 4.0):
                    self.get_logger().info(
                        f"shuting down after 5%f. iterations {(iteration)}")
                    rclpy.shutdown()
               
               N =20 # Number of iterations to achieve the desired goal, play with to change the speed

                # leg0,2,4
                # lifting in z-direction
                # myduration=time.Duration(seconds=0.001,nanoseconds=0.0)
               for k in range(6):
                legnumber=k  
                self.commands_Velocity_leg024.data[legnumber] = 0.001
                self.commands_Velocity_leg024.data[legnumber + 6] = 0.001
                self.commands_Velocity_leg024.data[legnumber+12] = 0.001
                xyz_from_base_to_world = self.FromWorldToRobotBase(
                    legnumber, self.leg0_xHome, self.leg0_yHome, self.leg0_zHome)
                xyz_from_world_to_base = self.FromRobotBaseToWorld(
                    legnumber, xyz_from_base_to_world[0], xyz_from_base_to_world[1], xyz_from_base_to_world[2] + lift_z)

                XGoal_leg0 = np.array(
                    [xyz_from_world_to_base[0], xyz_from_world_to_base[1], xyz_from_world_to_base[2]])
                deltaX_leg0 = np.array([0.0, 0.0, 0.0])
                PACt_leg0 = np.array([0.0, 0.0, 0.0])

                path_leg0 = np.array([0.0, 0.0, 0.0])
                qAct_leg0 = np.array(
                    [self.array_of_position[legnumber], self.array_of_position[legnumber +6], self.array_of_position[legnumber +12]])

                PStart_leg0 = np.array(
                    [self.leg0_xHome, self.leg0_yHome, self.leg0_zHome])
                PACt_leg0 = PStart_leg0

                i = 0
                for i in range(N):
                    start_time = self._clock.now().nanoseconds
                    path_leg0 = PStart_leg0 + \
                        (XGoal_leg0 - PStart_leg0)*(i)/(N-1)
                    deltaX_leg0 = path_leg0 - PACt_leg0

                    dq_leg0 = self.inverseKinematics(
                        qAct_leg0[0], qAct_leg0[1], qAct_leg0[2], deltaX_leg0[0], deltaX_leg0[1], deltaX_leg0[2])
                    qAct_leg0 = qAct_leg0 + dq_leg0
                    qAct_leg0 = self.limitCheck(
                        qAct_leg0[0], qAct_leg0[1], qAct_leg0[2])
                    self.commands_Position_leg024.data[legnumber] = qAct_leg0[0]
                    self.commands_Position_leg024.data[legnumber +
                                                       6] = qAct_leg0[1]
                    self.commands_Position_leg024.data[legnumber +
                                                       6 + 6] = qAct_leg0[2]
                    dummy = 0
                   
                    
                    while not self.default_callback_group.can_execute(self.position_pub):
                         self.get_logger().info(
                        f"cannot be executed {(0)}")
                   
                    if self.default_callback_group.beginning_execution(self.position_pub):  
                     self.position_pub.publish(self.commands_Position_leg024)
                     self.get_logger().info(
                        f"finishing {(0)}")
                    self.default_callback_group.ending_execution(self.position_pub)
                    self.get_logger().info(
                        f"publishing leg024 lift {(self.commands_Position_leg024)}")
                    PACt_leg0 = self.forwardKinematics(
                        qAct_leg0[0], qAct_leg0[1], qAct_leg0[2])

                # next -movement
                xyz_from_base_to_world = self.FromWorldToRobotBase(
                    legnumber, PACt_leg0[0], PACt_leg0[1], PACt_leg0[2])
                xyz_from_world_to_base = self.FromRobotBaseToWorld(
                    legnumber, xyz_from_base_to_world[0], xyz_from_base_to_world[1] + For_Y, xyz_from_base_to_world[2])

                XGoal_leg0 = np.array(
                    [xyz_from_world_to_base[0], xyz_from_world_to_base[1], xyz_from_world_to_base[2]])
                deltaX_leg0 = np.array([0.0, 0.0, 0.0])
                path_leg0 = np.array([0.0, 0.0, 0.0])

                PStart_leg0 = PACt_leg0
                i = 0
                for i in range(N):
                    start_time = self.get_clock().now
                    path_leg0 = PStart_leg0 + \
                        (XGoal_leg0 - PStart_leg0)*(i)/(N-1)
                    deltaX_leg0 = path_leg0 - PACt_leg0

                    dq_leg0 = self.inverseKinematics(
                        qAct_leg0[0], qAct_leg0[1], qAct_leg0[2], deltaX_leg0[0], deltaX_leg0[1], deltaX_leg0[2])
                    qAct_leg0 = qAct_leg0 + dq_leg0
                    qAct_leg0 = self.limitCheck(
                        qAct_leg0[0], qAct_leg0[1], qAct_leg0[2])

                    self.commands_Position_leg024.data[legnumber] = qAct_leg0[0]
                    self.commands_Position_leg024.data[legnumber +
                                                       6] = qAct_leg0[1]
                    self.commands_Position_leg024.data[legnumber +
                                                       6 + 6] = qAct_leg0[2]

                    while not self.default_callback_group.can_execute(self.position_pub):
                         self.get_logger().info(
                        f"cannot be executed {(0)}")
                   
                    if self.default_callback_group.beginning_execution(self.position_pub):  
                     self.position_pub.publish(self.commands_Position_leg024)
                     self.get_logger().info(
                        f"finishing {(0)}")
                    self.default_callback_group.ending_execution(self.position_pub)
                    
                    self.get_logger().info(
                        f"publishing leg024 next move {(self.commands_Position_leg024)}")
                    PACt_leg0 = self.forwardKinematics(
                        qAct_leg0[0], qAct_leg0[1], qAct_leg0[2])

                # leg024 move back to the origine z position
                xyz_from_base_to_world = self.FromWorldToRobotBase(
                    legnumber, PACt_leg0[0], PACt_leg0[1], PACt_leg0[2])
                xyz_from_world_to_base = self.FromRobotBaseToWorld(
                    legnumber, xyz_from_base_to_world[0], xyz_from_base_to_world[1], xyz_from_base_to_world[2]-lift_z)

                XGoal_leg0 = np.array(
                    [xyz_from_world_to_base[0], xyz_from_world_to_base[1], xyz_from_world_to_base[2]])

                deltaX_leg0 = np.array([0.0, 0.0, 0.0])
                path_leg0 = np.array([0.0, 0.0, 0.0])

                PStart_leg0 = PACt_leg0
                i = 0
                for i in range(N):
                    start_time = self.get_clock().now
                    path_leg0 = PStart_leg0 + \
                        (XGoal_leg0 - PStart_leg0)*(i)/(N-1)
                    deltaX_leg0 = path_leg0 - PACt_leg0

                    dq_leg0 = self.inverseKinematics(
                        qAct_leg0[0], qAct_leg0[1], qAct_leg0[2], deltaX_leg0[0], deltaX_leg0[1], deltaX_leg0[2])
                    qAct_leg0 = qAct_leg0 + dq_leg0
                    qAct_leg0 = self.limitCheck(
                        qAct_leg0[0], qAct_leg0[1], qAct_leg0[2])
                    self.commands_Position_leg024.data[legnumber] = qAct_leg0[0]
                    self.commands_Position_leg024.data[legnumber +
                                                       6] = qAct_leg0[1]
                    self.commands_Position_leg024.data[legnumber +
                                                       6 + 6] = qAct_leg0[2]

                    # time.sleep(0.01)
                    while not self.default_callback_group.can_execute(self.position_pub):
                         self.get_logger().info(
                        f"cannot be executed {(0)}")
                   
                    if self.default_callback_group.beginning_execution(self.position_pub):  
                     self.position_pub.publish(self.commands_Position_leg024)
                     self.get_logger().info(
                        f"finishing {(0)}")
                    self.default_callback_group.ending_execution(self.position_pub)
                    
                    self.get_logger().info(
                        f"publishing leg024 back home {(self.commands_Position_leg024)}")
                    PACt_leg0 = self.forwardKinematics(
                        qAct_leg0[0], qAct_leg0[1], qAct_leg0[2])
                '''self.leg0_q1_start=qAct_leg0[0]
            self.leg0_q2_start=qAct_leg0[1]
            self.leg0_q3_start=qAct_leg0[2]'''
                '''self.array_of_q[2]=qAct_leg2[0]
            self.array_of_q[8]=qAct_leg2[1]
            self.array_of_q[14]=qAct_leg2[2]
            self.array_of_q[4]=qAct_leg4[0]
            self.array_of_q[10]=qAct_leg4[1]
            self.array_of_q[16]=qAct_leg4[2]'''

                '''self.leg0_xHome=PACt_leg0[0]
            self.leg0_yHome=PACt_leg0[1]
            self.leg0_zHome=PACt_leg0[2]'''

                '''self.leg2_xHome=PACt_leg2[0]
            self.leg2_yHome=PACt_leg2[1]
            self.leg2_zHome=PACt_leg2[2]
            self.leg4_xHome=PACt_leg4[0]
            self.leg4_yHome=PACt_leg4[1]
            self.leg4_zHome=PACt_leg4[2]'''

                # leg1,3,5

                # For_Y+=0.02

        except KeyboardInterrupt:
            pass

    def my_callback(self, msg: JointState):

        if self.counter_for_init_jointsValue == 0:
            print("\n")
            self.get_logger().info(f" start velocity\n {(msg)} ")
            for i in range(18):
                self.array_of_position[i]=msg.position[i]
            self.leg0_q1_start=msg.position[0]
            self.leg0_q2_start=msg.position[6]
            self.leg0_q3_start=msg.position[12]
            self.leg2_q1_start=msg.position[2]
            self.leg2_q2_start=msg.position[8]
            self.leg2_q3_start=msg.position[14]
            self.leg4_q1_start=msg.position[4]
            self.leg4_q2_start=msg.position[10]
            self.leg4_q3_start=msg.position[16]
            self.leg1_q1_start=msg.position[1]
            self.leg1_q2_start=msg.position[7]
            self.leg1_q3_start=msg.position[13]
            self.leg3_q1_start=msg.position[3]
            self.leg3_q2_start=msg.position[9]
            self.leg3_q3_start=msg.position[15]
            self.leg5_q1_start=msg.position[5]
            self.leg5_q2_start=msg.position[11]
            self.leg5_q3_start=msg.position[17]
            self.setStartXYZPosition()

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
           self.commands_Position_legStart.data[17]=self.q3_start'''
            # self.position_pub.publish( self.commands_Position_legStart)
        self.counter_for_init_jointsValue += 1

    def pu_call_back(self):
        self.get_logger().info(f" I missed my deadline \n {( -1)} ")

    def fillValuesInTheArray(self, legnumber):
        if (legnumber % 2) == 1:

            self.commands_Position_leg024.data[legnumber] = self.array_of_q[legnumber]
            self.commands_Position_leg024.data[legnumber +
                                               6] = self.array_of_q[legnumber + 6]
            self.commands_Position_leg024.data[legnumber +
                                               12] = self.array_of_q[legnumber + 12]
        elif (legnumber % 2) == 0:
            self.commands_Position_leg135.data[legnumber] = self.array_of_q[legnumber]
            self.commands_Position_leg135.data[legnumber +
                                               6] = self.array_of_q[legnumber + 6]
            self.commands_Position_leg135.data[legnumber +
                                               12] = self.array_of_q[legnumber + 12]

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

    def inverseKinematics(self, q1, q2, q3, dx, dy, dz):
        dq = np.array([0.0, 0.0, 0.0])
        q = np.array([q1, q2, q3])
        det = self.DetOfJacobian(q1, q2, q3)
        self.get_logger().info(f" determinant of the jacobian \n {( det)} ")
        dq[0] = (dy*cos(q1) - dx*sin(q1))/(l1 + l3*cos(q2 + q3) + l2*cos(q2))
        dq[1] = -(1.0*(dz*cos(q2)*sin(q3) + dz*cos(q3)*sin(q2) - dx*cos(q1)*cos(q2)*cos(q3) - dy*cos(q2)
                  * cos(q3)*sin(q1) + dx*cos(q1)*sin(q2)*sin(q3) + dy*sin(q1)*sin(q2)*sin(q3)))/(l2*sin(q3))
        dq[2] = (dz*l2*sin(q2) - dx*l2*cos(q1)*cos(q2) - dy*l2*cos(q2)*sin(q1) + dz*l3*cos(q2)*sin(q3) + dz*l3*cos(q3)*sin(q2) - dx*l3*cos(q1)
                 * cos(q2)*cos(q3) - dy*l3*cos(q2)*cos(q3)*sin(q1) + dx*l3*cos(q1)*sin(q2)*sin(q3) + dy*l3*sin(q1)*sin(q2)*sin(q3))/(l2*l3*sin(q3))
        for i in range(3):
            if abs(dq[i]) < 1e-10:
                dq[i] = 0.0
            if dq[i] > 3.14:
                dq[i] = q[i]
            if dq[i] < -3.14:
                dq[i] = q[i]
        self.get_logger().info(f" displacement angles dq \n {( dq)} ")
        return dq

    def controllerError(self, msg: JointState):
        temp_error = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.get_logger().info(
            f" current joint values   \n {( self.array_of_q)} ")
        for i in range(18):
            temp_error[i] = self.array_of_q[i]-msg.position[i]
            if abs(temp_error[i]) > 0.00000000001:
                self.array_of_q[i] += temp_error[i]
        self.get_logger().info(f" controller error  \n {( temp_error)} ")

    def inverseKinematicsTrigo(self, x, y, z):
        dq = np.array([0.0, 0.0, 0.0])

        ll = sqrt(pow(x, 2)+pow(y, 2))
        x2 = pow(x, 2)
        y2 = pow(y, 2)

        gammaRadians = -atan2(x, y)
        aux = sqrt(x2+y2)-coxa
        HF = sqrt(pow(ll-coxa, 2)+pow(z, 2))
        alpha1 = atan2((ll-coxa), z)
        alpha2 = acos(((pow(tibia, 2))-pow(femur, 2)-pow(HF, 2))/(-2*femur*HF))
        alphaRadians = (pi/2)-(alpha1+alpha2)
        betaRadians = acos((pow(tibia, 2)+pow(femur, 2) -
                           pow(z, 2)-pow(aux, 2))/(-2*femur*tibia))
        dq[0] = gammaRadians*180/pi
        dq[1] = alphaRadians*180/pi
        dq[2] = betaRadians*180/pi
        self.get_logger().info(f" qs angles \n {( (dq))} ")

        return dq
    
    def setStartXYZPosition(self):
       for i in range(6):
          xack=self.forwardKinematics(self.array_of_q[i],self.array_of_q[i+6],self.array_of_q[i+12])  
          self.array_of_position[i]=xack[0]
          self.array_of_position[i+6]=xack[1] 
          self.array_of_position[i+12]=xack[2]

    def FromRobotBaseToWorld(self, leg_number, xw, yw, zw):
        xyz_in_robot_base = np.array([0.0, 0.0, 0.0])
        if leg_number == 0:
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

    def forwardKinematics(self, q1, q2, q3):
        xAck = np.array([0.0, 0.0, 0.0])

        xAck[0] = 0.5*l2*cos(q1 - 1.0*q2) + 0.5*l2*cos(q1 + q2) + 0.5 * \
            l3*cos(q2 - 1.0*q1 + q3) + l1*cos(q1) + 0.5*l3*cos(q1 + q2 + q3)
        xAck[1] = 0.5*l2*sin(q1 - 1.0*q2) + 0.5*l2*sin(q1 + q2) - 0.5 * \
            l3*sin(q2 - 1.0*q1 + q3) + l1*sin(q1) + 0.5*l3*sin(q1 + q2 + q3)
        xAck[2] = - 1.0*l3*sin(q2 + q3) - 1.0*l2*sin(q2)
        for i in range(3):
            if abs(xAck[i]) < 0.000000001:
                xAck[i] = 0.0
        print("\n")
        self.get_logger().info(f"Actual cartesian position  \n {( xAck)} ")
        return xAck

    def DetOfJacobian(self, q1, q2, q3):
        det = 1.0*l2*l2*l3*sin(q2 + q3)*cos(q1)*cos(q1)*cos(q2)*cos(q2) - 1.0*l2*l3*l3*cos(q2 + q3)*cos(q2 + q3)*cos(q1)*cos(q1)*sin(q2) - 1.0*l2*l3*l3*cos(q2 + q3)*cos(q2 + q3)*sin(q1)*sin(q1)*sin(q2) + 1.0*l2*l2*l3*sin(q2 + q3)*cos(q2)*cos(q2)*sin(q1)*sin(q1) + 1.0*l2*l3*l3*cos(q2 + q3)*sin(q2 + q3)*cos(q2)*sin(q1)*sin(q1) - 1.0*l2*l2*l3*cos(q2 + q3)*cos(q1)*cos(
            q1)*cos(q2)*sin(q2) - 1.0*l2*l2*l3*cos(q2 + q3)*cos(q2)*sin(q1)*sin(q1)*sin(q2) - 1.0*l1*l2*l3*cos(q2 + q3)*cos(q1)*cos(q1)*sin(q2) + 1.0*l1*l2*l3*sin(q2 + q3)*cos(q1)*cos(q1)*cos(q2) - 1.0*l1*l2*l3*cos(q2 + q3)*sin(q1)*sin(q1)*sin(q2) + 1.0*l1*l2*l3*sin(q2 + q3)*cos(q2)*sin(q1)*sin(q1) + 1.0*l2*l3*l3*cos(q2 + q3)*sin(q2 + q3)*cos(q1)*cos(q1)*cos(q2)
        return det


def main():
    myController = Controller()


if __name__ == '_main_':
    main()
