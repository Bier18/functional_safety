# progetto_mobile_robotics
1. Preparazione
**Scegliere una cartella a piacere e aprire il terminale** (Se si è su Windows eseguire `wsl`)
**Clonare la repository con il comando**:
git clone https://github.com/Bier18/progetto_mobile_robotics.git
Aprire Docker Desktop
Aprire la cartella progetto_mobile_robotics in Visual Studio Code
Aprire tre terminali in Visual Studio Code
Eseguire `wsl` in ciascuno dei tre
2. Creazione immagine e container (primo terminale)
**Creare l’immagine Docker eseguendo**:
cd ./docker_ws
./build_progetto.sh
Aspettare il completamento dell’operazione
**Creare il container eseguendo**:
cd ..
./run.sh
**Portarsi nella cartella ros_workspace eseguendo**:
cd /root/ros_workspace
**Compilare i pacchetti eseguendo**:
colcon build --cmake-clean-cache
**Caricare l’ambiente eseguendo**:
source install/setup.bash
3. Avvio del container negli altri terminali
**Eseguire nei terminali 2 e 3 (prima sul secondo poi sul terzo)**:
./exec.sh
source install/setup.bash
4. Avvio simulazione (primo terminale)
**Impostare la variabile d’ambiente TURTLEBOT3_MODEL eseguendo**:
export TURTLEBOT3_MODEL=burger
**Lanciare il programma principale eseguendo**:
ros2 launch turtlebot3_gazebo empty_world.launch.py
5. Visualizzazione in RViz (secondo terminale)
**Avviare Rviz eseguendo**:
rviz2
**Impostare la visualizzazione dei robot in Rviz eseguendo**:
1. Cliccare su Add in basso a sinistra
2. Cliccare su By display type in alto a sinistra nella finestra appena aperta
3. Scorrere nel menù e selezionare RobotModel → cliccare OK
4. Nel menù a sinistra espandere RobotModel
5. Cliccare su Description Topic → scegliere /t0/robot_description
6. Ripetere i passaggi per gli altri 12 robot cambiando il namespace
**Aggiungere i marker LED eseguendo**:
1. Ripetere i primi 4 passaggi ma selezionare Marker invece di RobotModel
2. Espandere Marker nel menù a sinistra
3. Cliccare su Topic e inserire:
/led_marker
6. Nodo word_coder (terzo terminale)
**Lanciare il nodo word coder eseguendo**:
ros2 run utilities word_coder --ros-args -p word:=PAROLA_DA_CODIFICARE
