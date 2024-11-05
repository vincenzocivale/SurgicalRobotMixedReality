#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Pose
from std_msgs.msg import String, Float64MultiArray

class JoystickNode:
    def __init__(self, groups): 
        
        rospy.init_node('joystick_node', anonymous = True) 
        
        self.groups = groups
        self.group_topics = ['/' + element for element in groups]

        self.current_group_index = 0 
        
        self.target_pose_pub = rospy.Publisher(self.group_topics[self.current_group_index], Float64MultiArray, queue_size = 100)
        
        self.group_pub = rospy.Publisher('/current_group', String, queue_size = 100)
        self.group_pub.publish(self.groups[self.current_group_index])
        #rospy.loginfo(f"Currently selected group: {self.groups[self.current_group_index]}")

        self.arm_group_pose = Float64MultiArray()
        self.end_effector_movement = Float64MultiArray()
        self.x = 0
        self.y = 0
        self.z = 0

        self.joy_subscriber = rospy.Subscriber('joy', Joy, self.joy_callback)
        
        self.timer = rospy.Timer(rospy.Duration(1), self.publish_current_group) 
        
        self.publish_interval = 0.1
        self.last_publish_time = rospy.get_time()
        
        self.previous_buttons = [0] * 12
        
    def publish_current_group(self, event):
        self.group_pub.publish(self.groups[self.current_group_index])
        rospy.loginfo(f"Currently selected group: {self.groups[self.current_group_index]}")
    
    def joy_callback(self, data):           
        if data.buttons[5] == 1 and self.previous_buttons[5] == 0:
            self.change_group("next")
        elif data.buttons[4] == 1 and self.previous_buttons[4] == 0:
            self.change_group("previous")
        
        self.previous_buttons = list(data.buttons).copy()
        
        current_time = rospy.get_time()
        if current_time - self.last_publish_time >= self.publish_interval:
        
            if self.group_topics[self.current_group_index] == "/arm_group":
                self.update_arm_group_pose(data)
        
            elif self.group_topics[self.current_group_index] == "/end_effector_group":
                self.update_end_effector_group_pose(data)
        
            else:
                rospy.loginfo("Warning! Selected group doesn't exist!")
                
            self.last_publish_time = current_time
        
    def change_group(self, direction):
        if direction == "next":
            self.current_group_index = (self.current_group_index + 1) % len(self.group_topics)
        elif direction == "previous":
            self.current_group_index = (self.current_group_index - 1) % len(self.group_topics)

        self.group_pub.publish(self.groups[self.current_group_index])
        self.target_pose_pub = rospy.Publisher(self.group_topics[self.current_group_index], Float64MultiArray, queue_size = 100) 
       
    def update_arm_group_pose(self, data):

        # Modifica la posizione basata sugli stick
        self.x += data.axes[0] * 0.005  # Viene incrementata/decrementata la posizione lungo l'asse X (muovere l'analogico sinistro verso destra o sinistra)
        self.y += data.axes[1] * 0.005  # Viene incrementata/decrementata la posizione lungo l'asse Y (muovere l'analogico sinistro verso l'alto o il basso)
        self.z += data.axes[3] * 0.005 # Viene incrementata/decrementata la posizione lungo l'asse Z (muovere l'analogico destro verso destra o sinistra)
        
        if self.x < -0.55:
            self.x = -0.55
        elif self.x > 0.55:
            self.x = 0.55
        if self.y < -0.20:
            self.y = -0.20
        elif self.y > 0.24:
            self.y = 0.24
        if self.z < -0.30:
            self.z = -0.30
        elif self.z > 0.35:
            self.z = 0.35
  
        self.arm_group_pose.data = [self.x, self.y, self.z]
        
        rospy.loginfo(f"Target position for group number {self.current_group_index + 1}: ({self.x}, {self.y}, {self.z})")

        self.target_pose_pub.publish(self.arm_group_pose) 
        
    def update_end_effector_group_pose(self, data):
        self.end_effector_movement.data = [0,0,0]
        # NOTA: Per controllare le traslazioni dell'end - effector si utilizza l'analogico sinistro muovendolo verso l'alto o verso il basso
        # NOTA: Per controllare le rotazioni dell'end - effector si utilizzano i tasti L2 ed R2
        
        # GESTIONE DELLE TRASLAZIONI
        if data.axes[1] > 0:
            self.end_effector_movement.data[0] = 0.5 
        elif data.axes[1] < 0: 
            self.end_effector_movement.data[0] = -0.5
        else: # Altrimenti...
            self.end_effector_movement.data[0] = 0 
            
        # GESTIONE DELLE ROTAZIONI DEL TOOL
        if data.buttons[6] == 1: 
            self.end_effector_movement.data[1] = -1 
        elif data.buttons[7] == 1:
            self.end_effector_movement.data[1] = 1
        else: # Altrimenti...
            self.end_effector_movement.data[1] = 0
            
        # GESTIONE DELLE ROTAZIONI DELL'END-EFFECTOR
        if data.buttons[3] == 1: 
            self.end_effector_movement.data[2] = -0.5 
        elif data.buttons[1] == 1: 
            self.end_effector_movement.data[2] = 0.5 
        else:
            self.end_effector_movement.data[2] = 0        
        
        self.target_pose_pub.publish(self.end_effector_movement)
        rospy.loginfo(f"Translational or angular velocity for group number {self.current_group_index + 1}: {self.end_effector_movement}")
            
if __name__ == "__main__":
    groups = ['arm_group', 'end_effector_group'] 
    joystick_input_node = JoystickNode(groups)

    try:
        rospy.spin() 
    except rospy.ROSInterruptException:
        pass
