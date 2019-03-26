#!/bin/bash
cd catkin_ws
catkin_make
cd ../
gnome-terminal -x bash -c "source ~/catkin_ws/devel/setup.bash; roslaunch kinova_bringup kinova_robot.launch kinova_robotType:=j2n6s300; read -n1"
sleep 3
gnome-terminal -x bash -c "source ~/catkin_ws/devel/setup.bash; roslaunch j2n6s300_moveit_config j2n6s300_demo.launch; read -n1"
sleep 7
gnome-terminal -x bash -c "source ~/catkin_ws/devel/setup.bash; rosrun kinova_arm_moveit_demo pick_place; read -n1"


