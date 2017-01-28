#!/bin/bash


tab=" --tab-with-profile=Default"
options=(--tab --title=Terminal)

cmds[1]="roslaunch ur_modern_driver ur5_bringup.launch limited:=true robot_ip:=192.168.1.102"
titles[1]="robot_ip"

cmds[2]="roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch limited:=true"
titles[2]="test2"

cmds[3]="roslaunch ur5_moveit_config moveit_rviz.launch config:=true"
titles[3]="test2"

cmds[4]="roslaunch pr2_moveit_config warehouse.launch moveit_warehouse_database_path:=~/moveit_db"
titles[4]="test2"


for i in 1 2 3 4; do
  options+=($tab --title="${titles[i]}"  -e "bash -c \"${cmds[i]} ; bash\"" )          
done

gnome-terminal "${options[@]}"


exit 0