Система планирования пути для квадрокоптера

В реализации использованы следующие проекты:  
https://github.com/introlab/rtabmap  
https://github.com/introlab/rtabmap_ros (взаимодействие RTAB-Map с ROS)  
https://github.com/raulmur/ORB_SLAM2 
https://github.com/ethz-asl/mav_control_rw/ (контроллер)  
https://github.com/ethz-asl/ethzasl_msf (объединение сенсоров)  
https://github.com/AlessioTonioni/Autonomous-Flight-ROS/ (пакеты action_controller и moveit_simple_controller_manager)  
https://github.com/ethz-asl/rotors_simulator
  
А также стандартные средства ROS:  
https://github.com/ros-simulation/gazebo_ros_pkgs  
https://github.com/ros-planning/moveit  

Установка:

1. Установить среду ROS версии Kinetic для Ubuntu 16.04
2. Создать рабочее окружение (workspace, catkin_ws)
3. Установить catkin_tools: http://catkin-tools.readthedocs.io/en/latest/installing.html
4. Скачать и установить ORB_SLAM (по инструкции в официальном репозитории)
5. Скачать и установить RTAB-Map
4. Скачать проект:  
   cd catkin_ws/src  
   git clone https://github.com/vkpankov/mav_path_planner  
5. Построить проект: catkin build

Видео-демонстрация работы (ускоренная):  
https://youtu.be/5_vqReFoaeU
