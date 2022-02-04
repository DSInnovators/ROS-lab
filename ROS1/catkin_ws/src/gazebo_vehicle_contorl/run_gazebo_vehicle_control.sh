echo "running model_push"

roscore &
sleep 10

rosrun gazebo_ros gzserver  src/gazebo_vehicle_contorl/worlds/vehicle_control.world --verbose &
sleep 10

gzclient &

sleep 10
rviz rviz -f camera_link &

sleep 10
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=vehicle/cmd_vel