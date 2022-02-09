#!/bin/bash

# echo "Creating a virtual environment object_detection_venv"
# python3 -m venv subscribers/object-detection/object_detection_venv 
# echo "###########################################"

# echo "Activating venv"
# . subscribers/object-detection/object_detection_venv/bin/activate
# echo "###########################################"

# echo "Install requirements"
# pip3 install -r subscribers/object-detection/requirements.txt
# echo "###########################################"

# echo "Run object detection"
# echo "###########################################"
# echo "PRESS q TO TERMINATE THE PROGRAM"
# echo "###########################################"
. subscribers/object-detection/object_detection_venv/bin/activate
key="q"
python3 subscribers/object-detection/object_detection.py &
pid=$!
while read -n1 char ; do
  if [ "$char" = "$key" ] ; then
    kill "$pid"
    break
  fi
done