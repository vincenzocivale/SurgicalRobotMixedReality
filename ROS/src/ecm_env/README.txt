https://moveit.github.io/moveit_tutorials/

DIPENDENZE DA SCARICARE:
1) sudo apt-get install ros-noetic-joint-trajectory-controller
2) sudo apt-get install ros-noetic-joint-state-controller
3) sudo apt-get install ros-noetic-moveit
4) sudo apt-get install ros-noetic-joy
5) sudo apt-get update
6) sudo apt-get upgrade
7) catkin_make
8) source devel/setup.bash

DI SEGUITO VENGONO DESCRITTI I PRINCIPALI FILE PRESENTI ALL'INTERNO DEI PACCHETTI "ECM_DESCRIPTION" E "ECM_MOVEIT". I FILE NON CITATI NON SONO DIRETTAMENTE UTILI/INTERESSATI. 

ecm_description/
	/config
		/joint_trajectory_controller.yaml --> File in cui vengono definiti i controller che gestiscono i movimenti dei giunti del braccio robotico e dell'end effector.
	/meshes
		/example.STL --> Mesh in formato .STL di ciascun componente del robot.
	/urdf
		/ecm_robot.urdf --> File che contiene informazioni sulla morfologia del robot ecm, nonché le proprietà fisiche, visive e di collisione (ottenuto tramite il codice yaml2urdf.py).
	/launch
		/rviz_visualize_ecm.launch --> Permette di visualizzare il robot su RVIZ e simulare il movimento dei giunti pubblicando valori fittizi.
		/gazebo_visualize_gazebo.launch --> Permette di visualizzare il robot su GAZEBO.
		
ecm_moveit/
	/config
		/ros_controllers.yaml --> File che moveit utilizza per sapere quali controller devono ricevere i comandi e quali giunti devono essere mossi.
	/launch
		/ecm_visualize.launch --> Permette di visualizzare il robot su RVIZ (inclusa l'interfaccia di Moveit) e su Gazebo. Inoltre, permette di muovere il robot in delle pose predefinite.
		/joystick_control.launch --> Permette di controllare tramite joystick alcuni movimenti del robot.
						

AVVIARE SIMULAZIONE DEL ROBOT ECM E CONTROLLARNE I MOVIMENTI UTILIZZANDO JOYSTICK:
1) Collegare il joystick al pc
2) Nella parte superiore della macchina virtuale andare in "Dispositivi" e successivamente in "USB" e pinnare il joystick.
3) (new terminal) jstest /dev/input/js0 oppure .../js1 oppure /js2 e vedere a quale porta è collegato il joystick.
4) Aprire il file joystick_control.launch e alla riga di codice 4 inserire la porta corretta.
5) (new terminal) cd catkin_ws
6) catkin_make
7) source devel/setup.bash
8) roslaunch ecm_moveit ecm_visualize.launch
9) (new terminal) source devel/setup.bash
10) roslaunch ecm_moveit joystick_control.launch (per controllare il robot con il joystick)
11) Muovere il robot
12) Premere il tasto "quadrato" del joystick per fare il planning della traiettoria migliore
13) Premere il tasto "cerchio" del joystick per eseguire la traiettoria


		
MOVIMENTI PROVATI E VERIFICATI CON JOYSTICK:
- L'analogico sinistro, mosso in alto e in basso, permette di muovere rispettivamente in avanti e in dietro il braccio robotico lungo l'asse Y del sistema di riferimento globale.
- L'analogico sinistro, mosso verso destra e verso sinistra, permette di muovere in alto e in basso il braccio robotico lungo l'asse Z del sistema di riferimento globale.
- R1 ed L1 permettono la rotazione intorno all'asse X del sistema di riferimento globale

	


