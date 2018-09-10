# Installation guidelines for using CVIT-Robot
Documentation on how to install various dependencies required by our environment to run the robot

# ROS(Kinetic Kame) 
Ros is used for path planning and communication between modules
## For Ubuntu/Debian
Ros Kinetic has a size of 1GB. For further doubts follow this link -> http://wiki.ros.org/kinetic/Installation/Debian

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
apt-cache search ros-kinetic
sudo rosdep init
rosdep update
sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential build-essential cmake pkg-config

```
## For other OS 
Follow this link -> http://wiki.ros.org/kinetic/Installation


# Activating Ros 
We are setting an alias to be used. Before running any module run this alias.
```
sudo chmod +x /opt/ros/kinetic/setup.bash    
printf '\nalias ros_activate="source /opt/ros/kinetic/setup.bash;source %s/catkin_ws/devel/setup.bash"' "$BUILD_DIR"   >>  ~/.bashrc
source ~/.bashrc
```

# Use kinect2 using libfreenect2
USB 3 is a must to connect to the kinect2 
For ubuntu OS > 14.04 use this/installation script else read the repo's readme(link below) for installation
```
git clone https://github.com/OpenKinect/libfreenect2.git
cd libfreenect2
git clone https://github.com/OpenKinect/libfreenect2.git
cd libfreenect2
sudo apt-get install libusb-1.0-0-dev libturbojpeg0-dev libglfw3-dev
cmake . -Dfreenect2_DIR=$HOME/freenect2/lib/cmake/freenect2 -DCMAKE_INSTALL_PREFIX=$HOME/freenect2
make
make install
sudo cp ./platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/
```

**For any issues read https://github.com/OpenKinect/libfreenect2**

### Install catkin_ws
Catkin workspace will hold all the inbuilt ros libraries
```
cd $BUILD_DIR
sudo apt-get install ros-kinetic-catkin
mkdir -p ~/catkin_ws/src
cd ./catkin_ws/
ros_activate
catkin_make
```

### Install iai_kinect2
iai_kinect connects libfreenect2 to ros
```
cd src 
git submodule add https://github.com/code-iai/iai_kinect2.git
cd iai_kinect2
rosdep install -r --from-paths .
cd $CATKIN_DIR
catkin_make -DCMAKE_BUILD_TYPE="Release"
```

#### Now the kinect can connect to your laptop and ros

# Move motors using pysabertooth

### Install pysabertooth 
 ```
 git clone https://github.com/MomsFriendlyRobotCompany/pysabertooth.git
cd pysabertooth/  
 sudo -H pip install pysabertooth
```

# Run Ros on JS
To connect to the UI/Applications/HTML using json we must create a server which converts ros messages to json  

# Install Ros bridge 
 - Before running Ros client we need a rosbridge to send json to ros

    http://wiki.ros.org/rosbridge_suite/Tutorials/RunningRosbridge

# Install Ros client
All js applications must install ros-client or roslibjs to interact with ros
Using this npm library
```
    https://www.npmjs.com/package/roslibjs-client
```

Note:- A quick hack to prevent the problem is commenting out topic unregistration in pulibhser.py 

**Location /opt/ros/kinetic/lib/python2.7/dist-packages/rosbridge_library/internal/publishers.py**


311 # if not self._publishers[topic].has_clients():

312 #           self._publishers[topic].unregister()

313 #            del self._publishers[topic] 


# Important Ros libraries 
```
sudo -Eapt-get install ros-kinetic-catkin
sudo -E apt-get install ros-kinetic-opencv3
sudo -E apt-get install ros-kinetic-rosbridge-server
sudo -E apt-get install ros-kinetic-rtabmap
sudo -E apt-get install ros-kinetic-rtabmap-ros
sudo -E apt-get install ros-kinetic-pointcloud-to-laserscan
sudo apt-get install ros-kinetic-move-base
sudo apt-get install ros-kinetic-teleop_twist_keyboard

```

# Install Mate Terminal(optional)
gnome-terminal does not support tab creation using command 
```
sudo apt-get install mate-terminal
```
