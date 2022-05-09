#!/bin/bash

echo "Running gazebo"
echo "######################"

if pgrep -x "gzserver" > /dev/null
then
    echo "Restarting Gazebo"
    killall gzserver &
    gnome-terminal -e "gazebo --verbose world/gazebo_ros_diff_drive_demo.world" 
else
    gnome-terminal -e "gazebo --verbose world/gazebo_ros_diff_drive_demo.world" 
fi

echo "Creating a virtual environment object_detection_venv"
python3 -m venv subscribers/object-detection/object_detection_venv 
echo "###########################################"

echo "Activating venv"
. subscribers/object-detection/object_detection_venv/bin/activate
echo "###########################################"

echo "Install requirements"
pip3 install -r subscribers/object-detection/requirements.txt
echo "###########################################"

echo "Run object avoidance"
echo "###########################################"
echo "PRESS q TO TERMINATE THE PROGRAM"
echo "###########################################"

. subscribers/object-detection/object_detection_venv/bin/activate
key="q"
python3 subscribers/object-detection/autonomous_object_avoiding.py &
pid=$!
while read -n1 char ; do
  if [ "$char" = "$key" ] ; then
    killall gzserver &
    kill "$pid"

    break
  fi
done 
