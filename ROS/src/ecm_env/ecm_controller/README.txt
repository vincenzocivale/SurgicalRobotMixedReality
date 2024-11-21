DIPENDENZE DA SCARICARE:
1) sudo apt-get install ros-noetic-joint-trajectory-controller
2) sudo apt-get install ros-noetic-joint-state-controller
3) sudo apt-get install ros-noetic-joy
4) pip install ikpy
5) sudo apt-get update
6) sudo apt-get upgrade
7) cd catkin_ws
8) catkin_make
9) source devel/setup.bash

DI SEGUITO VENGONO DESCRITTI I PRINCIPALI FILE PRESENTI ALL'INTERNO DEL PACCHETTO "ECM_CONTROLLER"
NOTA: I FILE NON CITATI NON SONO DIRETTAMENTE UTILI/INTERESSATI AI FINI DEL TUTORIAL

ecm_controller/
	/config
		/joint_trajectory_controller.yaml --> File in cui vengono definiti i controllori che permettono di gestire i movimenti dei giunti del braccio robotico e dell'end effector.
	/launch
		/robot_control.launch --> 1) Permette di visualizzare l'ecm (endoscopic camera manipulator) su GAZEBO.
					   2) Avvia il nodo che gestisce l'interazione con il joystick della PS4 e il nodo che permette il calcolo della cinematica inversa.
	/meshes
		/example.STL --> Mesh in formato .STL di ciascun componente del robot.
	/scripts
		/joystick.py --> Script python che permette di leggere i valori pubblicati dal joystick sul topic /joy e, in funzione dei tasti/analogici premuti, di inviare tali valori correttamente alla IK.
		/inverse_kinematics.py --> 1) Script che legge su topic differenti (si ha un topic per ciascun gruppo controllato) i valori in arrivo dal joystick.
		                           2) Viene calcolata la cinematica inversa e vengono pubblicati i valori di rotazione/traslazione dei giunti sui topic dei controllori.	
	/urdf
		/ecm_robot.urdf --> File che contiene informazioni sulla morfologia del robot ecm, nonché le proprietà fisiche, visive e di collisione (ottenuto tramite il codice yaml2urdf.py).

AVVIARE SIMULAZIONE DEL ROBOT ECM SU GAZEBO E CONTROLLARNE I MOVIMENTI UTILIZZANDO UN JOYSTICK PS4:
NOTA: I successivi passaggi valgono per un pc con sistema operativo linux (Ubuntu 20.04)

1) Collegare il joystick al pc tramite cavo USB.
2) (Aprire un nuovo terminale) Lanciare il comando jstest /dev/input/js0 oppure .../js1 oppure .../js2 e vedere a quale percorso fa riferimento il joystick.
3) Aprire il file robot_control.launch e modificare la riga di codice 9 inserendo il percorso corretto.
4) (Aprire un nuovo terminale) cd catkin_ws
5) catkin_make
6) source devel/setup.bash
7) roslaunch ecm_controller robot_control.launch
8) Muovere il robot utilizzando i pulsanti e gli analogici definiti sotto.

NOTA: Se si sta utilizzando una macchina virtuale, per collegare il joystick seguire i seguenti passaggi:
1) Collegare il joystick al pc tramite cavo USB.
2) Nella parte superiore della macchina virtuale andare in "Dispositivi", successivamente in "USB" e pinnare il joystick.
3) Ripartire dal punto 2 del caso precedente...

MAPPATURA DEL JOYSTICK:

- Pulsanti L1 ed R1 --> Permettono di cambiare il gruppo controllato (arm_group oppure end_effector_group)

1) Controllo del braccio robotico:
	- Analogico sinistro verso l'alto o verso il basso --> Permette di muovere il braccio in avanti o in dietro (lungo l'asse y)
	- Analogico sinistro verso sinistra o destra --> Permette di muovere il braccio a sinistra o destra (lungo l'asse x).
	- Analogico destro verso sinistr o destra --> Permette di muovere il braccio in alto o in basso (lungo l'asse z).

2) Controllo dell'end_effector:
	- Analogico sinistro verso l'alto o verso il basso --> Permette la traslazione del tool (lungo l'asse z).
	- Pulsanti L2 ed R2 --> Permettono la rotazione del tool (intorno all'asse z).
	- Pulsanti "Cerchio" e "Croce" --> Permettono la rotazione del supporto dell'end_effector (intorno all'asse x).



