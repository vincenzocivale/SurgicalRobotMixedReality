#!/usr/bin/env python3

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from std_msgs.msg import Float64, Float64MultiArray, String
import numpy as np
from ikpy.chain import Chain
import argparse
import time
import sys
import tty
from select import select
import termios

position = [0, 0, 0]
end_effector_translation = 0
end_effector_rotation = 0
end_effector_rotation2 = 0
group = ""

def listener():
    rospy.Subscriber('/current_group', String, group_callback)
    rospy.Subscriber('/arm_group', Float64MultiArray, arm_callback)
    rospy.Subscriber('/end_effector_group', Float64MultiArray, end_effector_callback)

def end_effector_callback(msg):
    global end_effector_translation, end_effector_rotation, end_effector_rotation2
    end_effector_translation = msg.data[0]
    end_effector_rotation = msg.data[1]
    end_effector_rotation2 = msg.data[2]

def group_callback(msg):
    global group
    group = msg.data
        
def arm_callback(msg):
    global position
    position = list(msg.data)
    position[2] = position[2] + 0.37

class Arm:
    def __init__(self):
        global position
        self.position = position
        self.current_joint_state = None 
        self.urdf_file_path = "/home/fgasta/catkin_ws/src/ecm_env/ecm_controller/urdf/ecm_robot.urdf"
        self.tolerance = 0.1

        self.pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size = 50) 
        rospy.Subscriber('/arm_controller/state', JointTrajectoryControllerState, self.joint_states_callback) 

        self.joint_trajectory_msg = JointTrajectory()
        self.kinematic_chain = None
        
        self.configure_kinematic_chain()

    def joint_states_callback(self, msg): 
        self.current_joint_state = msg

    def get_end_effector_position(self): 
        
        if self.current_joint_state is None:
            rospy.logwarn('Stato dei giunti non disponibile')
            return None

        joint_positions = self.current_joint_state.actual.positions
        end_effector_frame = self.kinematic_chain.forward_kinematics(
            joints=[0] + list(joint_positions) + [0, 0])  
        end_effector_position = end_effector_frame[:3, 3]
        
        return end_effector_position, joint_positions

    def configure_kinematic_chain(self): 
        mask = [False, True, True, True, True, False, False]
        self.kinematic_chain = Chain.from_urdf_file(self.urdf_file_path, active_links_mask=mask)

    def execute_arm_group(self):
        global position
        
        if position is None:
            rospy.logwarn('Posizione target non specificata per arm_group')
            return
         
        joint_values = self.kinematic_chain.inverse_kinematics(target_position=position)
        self.joint_trajectory_msg.joint_names = ['base_link__yaw_link', 'yaw_link__pitch_front_link',
                                                 'pitch_front_link__pitch_bottom_link', 'pitch_bottom_link__pitch_end_link']

        point = JointTrajectoryPoint()
        point.positions = joint_values[1:len(self.joint_trajectory_msg.joint_names)]
        point.time_from_start = rospy.Duration(1.0)

        self.joint_trajectory_msg.points = [point]
        rate = rospy.Rate(10)

        start_time = time.time()

        while not rospy.is_shutdown():
            current_time = time.time()
            self.pub.publish(self.joint_trajectory_msg)
        
            result = self.get_end_effector_position()
            if result is not None:
                current_position, _ = result
                #rospy.loginfo(f'Euclidean distance from target: {np.linalg.norm(current_position - np.array(position))}')
            
                if np.linalg.norm(current_position - np.array(position)) <= self.tolerance:
                    #rospy.loginfo(f'Target point successfully reached within the tolerance of: {self.tolerance} metri')
                    break
            else:
                rospy.logwarn('Joint state not available. Retrying...')

            if current_time - start_time > 5:
                #rospy.loginfo('Target point not reachable!')
                break

            rate.sleep()

class EndEffector(Arm):
    def __init__(self):
        super().__init__()
        self.vel_pub = rospy.Publisher('/trasl_end_effector_controller/command', Float64, queue_size = 50)
        self.rot_pub = rospy.Publisher('/rot_end_effector_controller/command', Float64, queue_size = 50)
        self.rot_pub2 = rospy.Publisher('/end_effector_controller_3/command', Float64, queue_size = 50)

    def execute_end_effector_group(self):
        global end_effector_translation, end_effector_rotation, end_effector_rotation2
        self.vel_pub.publish(end_effector_translation)
        self.rot_pub.publish(end_effector_rotation)
        self.rot_pub2.publish(end_effector_rotation2)

def main():
    global group
    rate = rospy.Rate(10)  
    # Vengono creati i controllori
    arm_controller = Arm()
    end_effector_controller = EndEffector()
    
    while not rospy.is_shutdown():
        if group == 'end_effector_group':
            end_effector_controller.execute_end_effector_group()
        else:
            arm_controller.execute_arm_group()
        
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('joint_publisher', anonymous=True)
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
    
    main()
