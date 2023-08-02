# Prerequisites

- Run the sh file to install all packages

```shell
chmod +x requirements.sh
./requirements.sh
```

- Or Run following command to install required packages.
Perception:
```shell
sudo apt install ros-noetic-octomap-rviz-plugins ros-noetic-octomap-server
pip install -r Autonomous_Driving_ws/src/perception/yolov5/src/yolov5/requirements.txt
```

Control:
```shell
sudo apt install ros-noetic-pid ros-noetic-robot-localization ros-noetic-smach-ros
```

Planning
```shell
sudo apt-get install ros-noetic-navigation
sudo apt-get install ros-noetic-teb-local-planner
sudo apt-get install ros-noetic-ackermann-msgs
```

# Getting Started

1. Use the following command to clone the repository
```shell
git clone git@gitlab.lrz.de:i2ros_g13/i2ros_g13_autonomous_driving.git --depth 1

```
2. Copy the src-folder to your repository and build it
3. Download the Unity Environment: https://syncandshare.lrz.de/getlink/fiEg9ocZ6Pc5iuEa4QqN1b/
4. Unzip the Unity file and copy the files to .../devel/lib/simulation/
5. run the following command to launch.
```shell
roslaunch simulation yolov5_simulation.launch 
```

  
The car will start driving along the generated global and local path.
The traffic rules will be followed correctly.

