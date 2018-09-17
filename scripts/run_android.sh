#!/bin/zsh	
# Run the android servers
if [ "$1" != "roscore" ];
then
	# SET VARIABLES
	MASTER_IP="http://localhost:11311" # Set the ip of the master URI

	IP_ADDRESS=$(ip route get 1 | awk '{print $NF;exit}')

	# Setting ROS variables
	echo "Setting Environment Variables..."

	ros_activate
	export ROS_MASTER_URI=$MASTER_IP
	export ROS_HOSTNAME=$IP_ADDRESS
	
	echo "Running roscore"
	mate-terminal -e 'zsh -c "roscore"' &
	sleep 2
fi

# Start the tour guide server
cd ../src/UI/TourGuide
mate-terminal --window -e 'zsh -c "echo Running TourGuide App; yarn start; sleep 3"' &

# Start HTML for webview
cd ../RobotControl/WebServer
mate-terminal --tab -e 'zsh -c "echo Running Message Passing WebServer; npm start;"' & 
# Start the robot-control app on expo
cd ../App/
mate-terminal --tab -e 'zsh -c "echo Running Message Passing App; npm start; sleep 10"' & 
