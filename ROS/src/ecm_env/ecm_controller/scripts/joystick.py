#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Pose
from std_msgs.msg import String, Float64MultiArray

class JoystickNode:
    def __init__(self, groups): # Il prametro "groups" deve essere una lista di stringhe, dove ogni stringa corrisponde al nome di un gruppo che si vuole controllare
        
        rospy.init_node('joystick_node', anonymous = True) # Si inizializza il nodo "joystick_node"
        
        self.groups = groups
        self.group_topics = ['/' + element for element in groups] # Si crea una lista di topic associati a ciascun gruppo passato in input
        
        # Indice del gruppo attualmente selezionato (si inizia controllando il primo gruppo della lista)
        self.current_group_index = 0 
        
        # Publisher per pubblicare la posizione target sul topic del gruppo attualmente selezionato
        self.target_pose_pub = rospy.Publisher(self.group_topics[self.current_group_index], Float64MultiArray, queue_size = 100)
        
        # Publisher per pubblicare il gruppo attualmente selezionato sul topic /current_group
        self.group_pub = rospy.Publisher('/current_group', String, queue_size = 100)
        self.group_pub.publish(self.groups[self.current_group_index])  # Viene pubblicato il nome del gruppo attualmente selezionato appena inizializzato
        
        # Variabili necessarie per la posizione target
        self.arm_group_pose = Float64MultiArray()
        self.end_effector_movement = Float64MultiArray()
        self.end_effector_movement.data = [0,0,0]
        self.x = 0
        self.y = 0
        self.z = 0

        # Si crea un Subscriber che legge i comandi pubblicati dal joystick sul topic /joy e, alla ricezione di un messaggio, viene chiamata la funzione "joy_callback"
        self.joy_subscriber = rospy.Subscriber('joy', Joy, self.joy_callback)
        
        self.timer = rospy.Timer(rospy.Duration(1), self.publish_current_group) # Ogni 1 secondo viene chiamato il metodo "publish_current_group"
        
        # TODO: LE SUCCESSIVE DUE RIGHE DI CODICE SERVONO PER FARE IN MODO CHE IL JOYSTICK PUBBLICHI OGNI 0.1 SECONDI
        self.publish_interval = 0.1 # La posizione target data dal joystick viene pubblicata ogni 0.1 secondi
        self.last_publish_time = rospy.get_time() # Tempo dall'ultima pubblicazione
        
        self.previous_buttons = [0] * 12 # Variabile necessaria per memorizzare lo stato dei pulsanti del joystick, si inizializzato tutti e 12 i tasti come "non premuti"
        
    def publish_current_group(self, event): # Metodo che permette di pubblicare il gruppo attualmente selezionato, sul topic /current_group
        self.group_pub.publish(self.groups[self.current_group_index])
        rospy.loginfo(f"Currently selected group: {self.groups[self.current_group_index]}")
    
    def joy_callback(self, data):            
        # Controlla se il pulsante R1 o L1 è premuto per cambiare il gruppo controllato
        if data.buttons[5] == 1 and self.previous_buttons[5] == 0:  # Se il Pulsante R1 è premuto e prima non lo era...
            self.change_group("next") # Si passa al gruppo successivo
        elif data.buttons[4] == 1 and self.previous_buttons[4] == 0:  # Altrimenti, se il Pulsante L1 è premuto e prima non lo era..
            self.change_group("previous") # Si passa al gruppo precedente
        
        self.previous_buttons = list(data.buttons).copy() # Si aggiorna lo stato dei pulsanti del joystick
        
        # TODO: LE SUCCESSIVE DUE RIGHE DI CODICE SERVONO PERCHÈ I DATI DEL JOYSTICK VENGANO PUBBLICATI SOLO SE SONO PASSATI 0.1 SECONDI
        current_time = rospy.get_time()
        if current_time - self.last_publish_time >= self.publish_interval:
        
        # Se il gruppo attualmente selezionato è "arm_group", il codice aggiorna la posizione target del braccio
            if self.group_topics[self.current_group_index] == "/arm_group":
                self.update_arm_group_pose(data) # Viene aggiornata la posizione dell'arm in base ai valori restituiti dal joystick
        
        # Se il gruppo attualmente selezionato è "end_effector_group", il codice aggiorna la posizione target dell'end - effector
            elif self.group_topics[self.current_group_index] == "/end_effector_group":
                self.update_end_effector_group_pose(data)
        
        # Altrimenti, nel caso il gruppo selezionato non esista, viene loggato un messaggio di avviso
            else:
                rospy.loginfo("Warning! Selected group doesn't exist!")
        
        # TODO: LA RIGA DI CODICE SOTTOSTANTE SERVE PER INIZIALIZZARE IL TIMER UNA VOLTA CHE SONO STATI PUBBLICATI I DATI DEL JOYSTICK
            self.last_publish_time = current_time
        
    def change_group(self, direction):

        # Nelle righe di codice sottostanti viene cambiato l'indice del gruppo attualmente selezionato
        if direction == "next":
            self.current_group_index = (self.current_group_index + 1) % len(self.group_topics)
        elif direction == "previous":
            self.current_group_index = (self.current_group_index - 1) % len(self.group_topics)

        self.group_pub.publish(self.groups[self.current_group_index]) # Viene pubblicato il gruppo attualmente selezionato

        # Nella riga di codice sottostante, grazie a "self.current_group_index" viene inviata la posizione target al gruppo corretto
        self.target_pose_pub = rospy.Publisher(self.group_topics[self.current_group_index], Float64MultiArray, queue_size = 100) 
       
    def update_arm_group_pose(self, data): # Funzione necessaria per aggiornare la posizione dell'arm in base all'input del joystick

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
  
        self.arm_group_pose.data = [self.x, self.y, self.z] # Viene aggiornata la posizione di arm_group assegnando i nuovi valori di x,y e z
        
        rospy.loginfo(f"Target position for group number {self.current_group_index + 1}: ({self.x}, {self.y}, {self.z})")

        self.target_pose_pub.publish(self.arm_group_pose) # Viene pubblicata la nuova posizione target sul topic del gruppo d'interesse (in questo caso arm_group)
        
    def update_end_effector_group_pose(self, data): # Funzione necessaria per aggiornare la pose dell'end-effector in base all'input del joystick

        # NOTA: Per controllare le traslazioni dell'end - effector si utilizza l'analogico sinistro muovendolo verso l'alto o verso il basso
        # NOTA: Per controllare le rotazioni dell'end - effector si utilizzano i tasti L2 ed R2
        
        # GESTIONE DELLE TRASLAZIONI        
        if data.axes[1] > 0: # Se si sta spostando l'analogico sinistro verso l'alto...
            self.end_effector_movement.data[0] += 0.05 # Allora si trasla verso il basso l'end - effector
        elif data.axes[1] < 0: # Se si sta spostando l'analogico sinistro verso il basso...
            self.end_effector_movement.data[0] -= 0.05 # Allora si trasla verso l'alto l'end - effector
            
        # GESTIONE DELLE ROTAZIONI DEL TOOL INTORNO ALL'ASSE Z
        if data.buttons[6] == 1: # Se il pulsante L2 è premuto
            self.end_effector_movement.data[1] -= 0.025 # Allora si ruota in senso antiorario l'end - effector
        elif data.buttons[7] == 1: # Se il pulsante R2 è premuto
            self.end_effector_movement.data[1] += 0.025 # Allora si ruota in senso orario l'end - effector
            
        # GESTIONE DELLE ROTAZIONI DEL TOOL INOTRNO ALL'ASSE X
        if data.buttons[0] == 1: # Se il pulsante "Croce" è premuto
            self.end_effector_movement.data[2] -= 0.025 # Allora si ruota in senso antiorario l'end - effector
        elif data.buttons[1] == 1: # Se il pulsante "Cerchio" è premuto
            self.end_effector_movement.data[2] += 0.025 # Allora si ruota in senso orario l'end - effector
              
        self.target_pose_pub.publish(self.end_effector_movement) # Viene pubblicata la posizione dell'end - effector sul topic del gruppo d'interesse (in questo caso end_effector_group)
        # NOTA: Si ricorda che l'end - effector viene controllato con la cinematica diretta; quindi, non si fornisce in input una posizione da raggiungere ma quanto roto-traslare..
        rospy.loginfo(f"Translational or angular position for group number {self.current_group_index + 1}: {self.end_effector_movement}")
            
if __name__ == "__main__":
    groups = ['arm_group', 'end_effector_group']  # Lista contenente i gruppi di cui è costituito il robot
    joystick_input_node = JoystickNode(groups)

    try:
        rospy.spin()  # Keeps the node running and listening to topics
    except rospy.ROSInterruptException:
        pass
