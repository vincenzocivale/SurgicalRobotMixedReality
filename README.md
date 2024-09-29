# my_robot_project

Questo progetto ha come obiettivo la creazione di una simulazione di un robot chirurgico in realtà aumentata. Include vari nodi ROS per la gestione del controllo, la pianificazione della cinematica inversa e la pubblicazione della posizione del target.

## Struttura del Progetto

La struttura del progetto è organizzata come segue:

SurgicalRobotMixedReality/ │ ├── src/ │ ├── controller_node.py # Nodo del Controller │ ├── robot_node.py # Nodo del Robot │ ├── target_node.py # Nodo Target │ ├── launch/ # Directory per file di lancio │ ├── config/ # Configurazione generata con MoveIt │ ├── scripts/ │ ├── setup_robot.py # Script per configurare il robot (se necessario) │ └── test_robot.py # Script di test per funzionalità specifiche │ ├── README.md # Documentazione del progetto └── requirements.txt # Dipendenze del progetto (librerie Python)
