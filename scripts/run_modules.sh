#!/bin/zsh	
# This script runs python backend for ROS for message passing and everything 

#Usage ./run_python.sh

# use ./run_python.sh roscore  
#if you have already started roscore

# if [ "$1" != "roscore" ];
# then
# 	echo "Running roscore"
# 	mate-terminal -e 'zsh -c "roscore"' &
# 	sleep 2

# fi

echo "Running python files"

# Vision/Object detection and face recognition
cd ../src/vision
mate-terminal --window -e 'zsh -c "echo Running People Detection; python2 __init__.py"' &

sleep 2
cd ../speech
# Speech
mate-terminal --tab -e 'zsh -c "echo Running speaker; python2 listener.py"' 


sleep 2
cd ../AI
# AI
mate-terminal --tab -e 'zsh -c "echo Running visual_path_planning; python2 visual_path_planning.py"' --tab -e 'zsh -c "echo Running Tour guide backend; python2 tour_guide_data.py"'


sleep 2
cd ../movement
# AI
mate-terminal --tab -e 'zsh -c "echo Running Motors; python2 mover.py"'

# # # Running chartbot
# mate-terminal --tab -e 'zsh -c "echo Running chatbot;cd ~/ML/Chatbot-sentex/nmt_chatbot/; python3 user_input.py"' &
