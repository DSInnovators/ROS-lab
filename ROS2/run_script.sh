#!/bin/bash
echo "Running gazebo"
echo "######################"

if pgrep -x "gzserver" > /dev/null
then
    echo "Running"
    killall gzserver &
    gnome-terminal -e "gazebo --verbose world/gazebo_ros_diff_drive_demo.world" 
else
    gnome-terminal -e "gazebo --verbose world/gazebo_ros_diff_drive_demo.world" 
fi

gnome-terminal -e "python3 publishers/teleop_keyboard.py"
