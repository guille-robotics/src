En esta version, el mapa se genero, falta probar que el mapa se genere con el movimiento del robot, tenemos las publicaciones de los frames /odom /map y las de urdf que contienen a /laser

Para clonar este git se debe

- `mkdir my_ws`             # Crea un directorio llamado my_ws
- `cd my_ws`                 # Cambia al directorio  my_ws
- `mkdir src`                # Crea un directorio llamado src dentro de my_ws
- `cd src`                 # Cambia al directorio src
- `git clone <URL_del_repositorio>`   # Clona el repositorio especificado por la URL
- `cd my_ws`  # Vuelve al directorio  my_ws
- `colcon build` 


**Cosas que instalar**

- `sudo apt-get install ros-humble-navigation2 ros-humble-nav2-bringup`, para el SLAM
- `sudo apt-get install ros-humble-teleop-twist-keyboard`
- `sudo apt-get install ros-humble-slam-toolbox`, para el SLAM
- `sudo apt-get install ros-humble-xacro`
- `sudo apt-get install ros-humble-robot-localization`
- `sudo apt-get install ros-humble-joy`
- `sudo apt-get install ros-humble-joy-teleop`
- `sudo apt-get install ros-humble-tf-transformations`
- `sudo pip3 install transforms3d`
- [Paquete del rplidar](https://github.com/babakhani/rplidar_ros2)
- https://github.com/ros-drivers/usb_cam/tree/ros2




**Comandos a Ejecutar**

---- **Comandos que reemplazan a ROS2CONTROL** ----

- `ros2 launch robix_controller joystick_teleop.launch` para iniciar conexión con el joystick.
- `ros2 run robix_firmware simple_serial_transmitter.py` para enviar comandos de `/cmd_vel` a Arduino.
- `ros2 run robix_firmware velocidad_ruedas_publisher.py` para enviar datos de encoder desde Arduino hasta Raspberry.
- `ros2 run robix_firmware odometry_publisher.py` para publicar los datos de odometría.
- `ros2 run robix_firmware tf_transmitter.py` para generar la publicación de las transformadas desde odometría a map.

---- **Iniciar cuerpo del robot** ----

- `ros2 launch robix_description display.launch.py` inicia todas las transformadas.

---- **Iniciar el Rplidar** ----

- `ros2 launch rplidar_ros2 rplidar.launch.py`

  Si todo está bien con `ros2 topic`, debería visualizar los tópicos `/TF`, `/ODOM` y `/SCAN`.

---- **Iniciar el SLAM** ---- 
(Tutorial: [Generar un mapa con slam_toolbox](https://roboticsbackend.com/ros2-nav2-generate-a-map-with-slam_toolbox/))

- `ros2 launch nav2_bringup navigation_launch.py`

- `ros2 launch slam_toolbox online_async_launch.py`
- `rviz2 rviz2` (Hablitar TF, MAP,ODOMETRY) y selecionar fixed_frame "map"

---- **Extras** ----
- `https://www.youtube.com/watch?v=nOxywVabojk` , para instalar docker compose en Raspberry
- `#!/usr/bin/env python3`, para evitar el problema de OSError en los códigos de Python
- `https://fgcoca.github.io/Mis-notas-sobre-Linux-Ubuntu/ch340/`, dispositivos Serial CH340 Ubuntu 22.04




