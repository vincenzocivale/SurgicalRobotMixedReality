#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Pose
from std_msgs.msg import String

"""Il nodo JoystickInputNode riceve i dati dal joystick. A partire dalle informazioni ricevute:
- Cambia il gruppo di end-effector controllato (L1 e R1), pubblicando il gruppo attualmente selezionato sul topic 'current_group'.
- Aggiorna la posizione e l'orientamento dell'end-effector per il gruppo attualmente selezionato in base all'input del joystick.
- Pubblica la posizione e orientazione target dell'end-effector per il gruppo attualmente selezionato sul topic corrispondente, tra quelli target_group_topics.
"""

class JoystickInputNode:
    def __init__(self, target_group_topics):
        rospy.init_node('joystick_input_node', anonymous=True)

        # Lista dei topic per i diversi gruppi di end-effector che si vogliono controllare con il joystick
        self.joint_group_topics = target_group_topics

        # Indice del gruppo attualmente selezionato
        self.current_group_index = 0

        # Publisher per la posizione target dell'end-effector
        self.target_pose_pub = rospy.Publisher(self.joint_group_topics[self.current_group_index], Pose, queue_size=10)

        # Publisher per il gruppo attualmente selezionato e pubblica il gruppo iniziale
        self.group_pub = rospy.Publisher('current_group', String, queue_size=10)
        self.group_pub.publish(self.joint_group_topics[self.current_group_index])

        # Variabili per la posizione e orientamento target
        self.end_effector_pose = Pose()

        # Sottoscrittore per il joystick
        self.joy_subscriber = rospy.Subscriber('joy', Joy, self.joy_callback)

    def joy_callback(self, data):
        """
        Callback per elaborare l'input del joystick e aggiornare la posizione e l'orientamento
        dell'end-effector per il gruppo attualmente selezionato.
        """
        # Controlla se il pulsante R1 o L1 è premuto per cambiare il gruppo controllato
        if data.buttons[5] == 1:  # Pulsante R1
            self.change_group("next")
        elif data.buttons[4] == 1:  # Pulsante L1
            self.change_group("previous")

        # Aggiorna la posizione target solo se il joystick è inizializzato
        self.update_end_effector_pose(data)



    def change_group(self, direction):
        """
        Cambia il gruppo attualmente selezionato e aggiorna il publisher.
        """
        # Cambia l'indice del gruppo
        if direction == "next":
            self.current_group_index = (self.current_group_index + 1) % len(self.joint_group_topics)
        elif direction == "previous":
            self.current_group_index = (self.current_group_index - 1) % len(self.joint_group_topics)

        # Pubblica il gruppo attualmente selezionato
        self.group_pub.publish(self.joint_group_topics[self.current_group_index])

        # Pubblica la posizione target per il nuovo gruppo
        self.target_pose_pub = rospy.Publisher(self.joint_group_topics[self.current_group_index], Pose, queue_size=10)


    def update_end_effector_pose(self, data):
        """
        Aggiorna la posizione e l'orientamento dell'end-effector in base all'input del joystick.
        """
        # Modifica la posizione basata sugli stick
        self.end_effector_pose.position.x += data.axes[0] * 0.1  # Asse X
        self.end_effector_pose.position.y += data.axes[1] * 0.1  # Asse Y
        self.end_effector_pose.position.z += data.axes[3] * 0.1  # Asse Z

        # Log per la posizione target
        rospy.loginfo(
            f"Target Pose Group {self.current_group_index + 1}: Position({self.end_effector_pose.position.x}, "
            f"{self.end_effector_pose.position.y}, "
            f"{self.end_effector_pose.position.z})")

        # Pubblica la nuova posizione target sul topic del gruppo selezionato
        self.target_pose_pub.publish(self.end_effector_pose)



