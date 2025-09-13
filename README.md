## COME USARE LA REPOSIORY

**Si suppone che l'utente abbia installati docker e wsl sul proprio dispositivo**

1. Scegliere una cartella a piacere e aprire il terminale (Se si è su windows eseguire wsl)
2. clonare la repository con il comando:
  '''git clone https://github.com/Bier18/progetto_mobile_robotics.git'''
3. aprire docker desktop
4. aprire la cartella progetto_mobile_robotics  in visual studio code
5. aprire tre terminali in Visual Studio Code
6. eseguire wsl in ciascuno dei tre
7. usare il primo terminale per creare l'immagine docker eseguendo:
  '''cd ./docker_ws
     ./build_progetto.sh'''
8. aspettare il completamento dell'operazione
9. creare il container eseguendo
   '''cd ..
      ./run.sh'''
10. portarsi nella cartella ros_workspace eseguendo
    '''cd /root/ros_workspace'''
11. compilare i pacchetti eseguendo
    '''colcon build'''
12. caricare l'ambiente eseguendo
    '''source install/setup.bash'''
13. eseguire il container negli altri due terminali caricando l'ambiente in entrambi eseguendo (prima sul secondo poi sul terzo)
    '''./exec.sh
    source install/setup.bash'''
14. ritornare al primo container e impostare la variabile d'ambiente TURTLEBOT3_MODEL eseguendo
    '''export TURTLEBOT3_MODEL=burger'''
15. lanciare il programma principale eseguendo
    '''ros2 launch turtlebot3_gazebo empty_world.launch.py'''
16. passare al secondo terminale
17. avviare Rviz eseguendo
    '''rviz2'''
18. impostare la visualizzazione dei robot in Rviz eseguendo
    '''cliccare su Add in basso a sinistra'''
19. cliccare su By display type in alto a sinistra nella finestra appena aperta
20. scorrere nel menù aperto fino a trovare la voce RobotModel
21. cliccare ok
22. nel menù sulla sinistra comparirà la voce RobotModel
23. espandere la voce cliccando sulla freccetta a sinistra della voce stessa
24. cliccare sulla voce Description Topic
25. nella barra di testo che compare sulla destra cliccare la freccetta e selezionare il topic /t0/robot_description
26. ripetere tutti i passaggi altre dodici volte cambiando di volta in volta il nome del topic (cambia solo il namespace) fino a mostrare tutti i robot
27. ripetere i primi quattro passaggi selezionando la voce Marker al posto di RobotModel
28. nel menù sulla sinistra comparirà la voce Marker
29. espandere la voce cliccando sulla freccetta a sinistra della voce stessa
30. cliccare sulla voce Topic
31. nella barra di testo che compare sulla destra digitare /led_marker
32. spostarsi nel terzo terminale
33.lanciare il nodo word coder eseguendo
    '''ros2 run utilities word_coder --ros-args -p word:=(qui si deve digitare la parola da codificare)'''
