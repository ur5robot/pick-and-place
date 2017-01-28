#!/bin/bash

tab=" --tab-with-profile=Default"
options=(--tab --title=Terminal)

cmds[1]="roslaunch openni_launch openni.launch"
titles[1]="robot_ip"

cmds[2]="roslaunch ar_track_alvar ur5_bundle.launch"
titles[2]="test2"

cmds[3]="rosrun rviz rviz"
titles[3]="test2"



for i in 1 2 3; do
  options+=($tab --title="${titles[i]}"  -e "bash -c \"${cmds[i]} ; bash\"" )          
done

gnome-terminal "${options[@]}"


exit 0