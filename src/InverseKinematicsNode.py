#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray, String
from ikpy import chain
import numpy as np

"""Il nodo InverseKinematicsNode riceve i dati dal nodo JoystickInputNode. In particolare dal topic:
- 'current_group' per sapere il gruppo di end-effector controllato attualmente 
- 'target_group_topic': topic su cui vengono pubblicati i dati di posizione e orientamento target dell'end-effector attualmente selezionato.
  Per determinare quale target_group_topic è attualmente selezionato, il nodo InverseKinematicsNode si sottoscrive al topic 'current_group'.

Successivamente, il nodo InverseKinematicsNode calcola la cinematica inversa per il gruppo di end-effector attualmente selezionato e pubblica i valori delle articolazioni"""

class InverseKinematicsNode:

    def __init__(self, urdf_file):
        rospy.init_node('inverse_kinematics_node', anonymous=True)

        self.urdf_file = urdf_file

        # Inizializza il gruppo di end-effector
        self.target_group_topic = None

        # Inizializza la posizione e l'orientamento target
        self.target_pose = None

        # Sottoscrittore per determinare il topic dell'end-effector attualmente selezionato, e di conseguenza anche del gruppo selezionato
        self.group_sub = rospy.Subscriber('current_group', String, self.group_callback)

        # Sottoscrittore al topic per la posizione e orientamento target dell'end-effector
        self.target_sub = rospy.Subscriber(self.target_group_topic, Pose, self.target_callback)

        # Publisher per le articolazioni
        self.joint_pub = rospy.Publisher('target_configuration', Float64MultiArray, queue_size=10)



    def group_callback(self, data):
        """
        Callback per ricevere il gruppo di end-effector attualmente selezionato.
        """
        self.target_group_topic = data.data

    def target_callback(self, data):
        """
        Callback per ricevere la posizione e l'orientamento target dell'end-effector.
        """
        self.target_pose = data

    def calculate_joint_values(self):
        """
        Calculate the joint values for the currently selected end-effector group using ikpy.
        """
        if self.target_pose is None:
            rospy.logwarn("Target pose is not set.")
            return

        # Load the kinematic chain from the URDF file
        mask = [False, True, True, True, True, True, True]
        kinematic_chain = chain.Chain.from_urdf_file(self.urdf_file, active_link_mask = mask)

        # Convert the target pose to a transformation matrix
        orientation = self.target_pose.orientation
        position = self.target_pose.position

        # Calculate the inverse kinematics
        joint_values = kinematic_chain.inverse_kinematics(target_position = position) # Orientation da provare
        joint_values = joint_values[1:len(joint_values)] # Il primo valore è da escludere
        # Publish the joint values
        joint_values_msg = Float64MultiArray(data=joint_values)
        self.joint_pub.publish(joint_values_msg)


