Что сейчас сделано:

1)Простенькая моделька 4 колесного робота ( она пока что не так хороша, пока что она существует только для того, чтобы через gazebo я мог получать фейковые данные лазера)
2) tf_broadcaster - нужен только для того, чтобы данные лазера из локальной оси координат этого лазера, переводить их в координаты base_link - координаты робота, возможно вообще не понадобиться или сольется с чем-то другим
3) robot_odom - получая данные из joint-ов колес, конвертирую их в одометрические данные робота (позиция и скорость), а затем помещаю в топик odom и tf, чтобы затем их использовать для навигации
4) застрял на написании base controller

///Update
4)robot_gazebo_plugin - плагин для управления колесами робота в симляции
5)robot_teleop - управление роботом через джойстик
//собрать репозиторий
cd "rep"
catkin_make
source "rep"/devel/setup.bash

roslaunch robot_gazebo robot.launch 
roslaunch robot_teleop robot_teleop.launch 

визуализация в rviz 
roslaunch robot_description robot_rviz.launch 
///есть некоторые проблемы с поворачиваемостью - я как мог пытался подбирать коэффициенты трения

///TODO
5)подключить navigation stack 
6)написать программку, которая при завершении комманд с cmd_vel подключает navigation stack(например в автономном режиме возращает его в место инициализации)