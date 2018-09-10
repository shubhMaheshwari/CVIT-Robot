# This script can be used to install various dependencies required by our environment

# Store main dir as variables
CURRENT_DIR=$PWD
mkdir build
cd build
BUILD_DIR=$PWD

# ROS 
echo "Installing Ros Kinetic"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
apt-cache search ros-kinetic
sudo rosdep init
rosdep update
sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential

# Check which shell is running and make custom alias

if [ -n "$ZSH_VERSION" ]; then
    sudo chmod +x /opt/ros/kinetic/setup.zsh
    printf '\nalias ros_activate="source /opt/ros/kinetic/setup.zsh;source %s/catkin_ws/devel/setup.zsh"' "$BUILD_DIR"   >>  ~/.zshrc
    source ~/.zshrc 

elif [ -n "$BASH_VERSION" ]; then

    sudo chmod +x /opt/ros/kinetic/setup.bash    
    printf '\nalias ros_activate="source /opt/ros/kinetic/setup.bash;source %s/catkin_ws/devel/setup.bash"' "$BUILD_DIR"   >>  ~/.bashrc
    source ~/.bashrc
else
    echo "Unable to set rc file. Use bash, zsh"
    exit
fi

# Installing libfreenect2
echo "Installing libfreenect2"
cd $BUILD_DIR
git submodule add -f https://github.com/OpenKinect/libfreenect2.git
cd libfreenect2
sudo apt-get install libusb-1.0-0-dev libturbojpeg0-dev libglfw3-dev
cmake . -Dfreenect2_DIR=$HOME/freenect2/lib/cmake/freenect2 -DCMAKE_INSTALL_PREFIX=$HOME/freenect2
make
make install
sudo cp ./platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/


# Installing catwin workspace
echo "Installing catwin"
cd $BUILD_DIR
sudo apt-get install ros-kinetic-catkin
mkdir -p catkin_ws/src
cd ./catkin_ws/
ros_activate
catkin_make
ros_activate


CATKIN_DIR="$BUILD_DIR/catkin_ws"
#Install iai_kinect2
echo "Installing iai_kinect"
cd $CATKIN_DIR
echo $CATKIN_DIR
cd src 
git submodule add -f https://github.com/code-iai/iai_kinect2.git
cd iai_kinect2
rosdep install -r --from-paths .
cd $CATKIN_DIR
catkin_make -DCMAKE_BUILD_TYPE="Release"


# Install pysabertooth on python2
echo "Installing pysabertooth"
cd $BUILD_DIR
git submodule add -f https://github.com/MomsFriendlyRobotCompany/pysabertooth.git
cd pysabertooth/  
sudo -H pip2 install pysabertooth


# Other important libraries
sudo -E apt-get install ros-kinetic-catkin ros-kinetic-opencv3 ros-kinetic-rosbridge-server ros-kinetic-rtabmap ros-kinetic-rtabmap-ros ros-kinetic-pointcloud-to-laserscan ros-kinetic-move-base ros-kinetic-teleop_twist_keyboard

# Install mate-terminal(optional)
sudo apt-get install mate-terminal

cd $CURRENT_DIR