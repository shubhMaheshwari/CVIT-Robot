#!/bin/zsh	
#Usage ./run.sh


# This script runs backend for ROS 

# SET VARIABLES
MASTER_IP="http://localhost:11311" # Set the ip of the master URI

IP_ADDRESS=$(ip route get 1 | awk '{print $NF;exit}')

# Setting ROS variables
echo "Setting Environment Variables..."

ros_activate
export ROS_MASTER_URI=$MASTER_IP
export ROS_HOSTNAME=$IP_ADDRESS

sudo chmod 777 /dev/ttyACM0
sudo chmod 777 /dev/ttyACM1

# Starting ros
echo "Running roscore"
echo "Running rosrun for connecting to kinect"
echo "Running roslaunch for ros_bridge to connect with apps"

mate-terminal -e 'zsh -c "roscore"' &
sleep 2
mate-terminal  --tab -e 'zsh -c " roslaunch kinect2_bridge kinect2_bridge.launch publish_tf:=true; sleep 5"' --tab -e 'zsh -c "roslaunch rosbridge_server rosbridge_websocket.launch; sleep 10"' &

# Starting modules
./run_modules.sh roscore


# Start UI/Apps
./run_android.sh roscore



