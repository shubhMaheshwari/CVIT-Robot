# CVIT-Robot

# Video
![ Introduction Video ]( ./videos/intro.mp4 )

Using a kinect and wheelchair we are developing a robot which can interact with users. 

# File Structure and Modules
```
.
├── install         -> installation/setup guidelines
├── README.md 
├── scripts         -> bash scripts to run/install/test on robot
├── src
│   ├── AI          -> Control house for all modules
│   ├── movement    -> Control and recieve data from wheels
│   ├── speech      -> Listen and talk to users
│   ├── UI          -> Frontend to control robot
│   └── vision      -> Image processing 
├── tests           -> Testing of modules  
└── videos          
```

Each folder inside src is a module

## Getting started 
#### Clone the repository  
```
    git clone https://github.com/shubhMaheshwari/CVIT-Robot.git
```
#### Install 
Note -> Use source and not run ./install.sh
```
cd CVIT-Robot/scripts/
source ./install.sh
```
#### Run 
```
source ./run.sh
```


