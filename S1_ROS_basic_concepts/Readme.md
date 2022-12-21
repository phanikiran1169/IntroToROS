## Course Pre-Requisites

To complete this course work, the following will be needed
- Basic understanding of C++ and building software. [More info in course](https://sir.upc.edu/projects/rostutorials2021-22/2-development_tools/index.html#programming)
- A PC or laptop running with Ubuntu 20.04
- Software configuration management (Git). [More info in course](https://sir.upc.edu/projects/rostutorials2021-22/2-development_tools/index.html#version-control-using-git) 

## ROS Noetic Installation

### Regular system update
```
sudo apt-get update
````

### Terminator and Xterm
```
sudo apt install terminator
sudo apt install xterm
```

### ROS

Installation instructions of ROS noetic for ubuntu. [Detailed info](http://wiki.ros.org/noetic/Installation/Ubuntu)

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl gnupg  # if you haven't already installed curl and gnupg
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full
sudo apt update
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo rosdep init
rosdep update
```

Check if installation is successful or not through ROS environment variables
```
printenv | grep ROS
```

or run **roscore**
```
roscore
```

### Other ROS packages

### ROS Control
```
sudo apt install ros-noetic-ros-control ros-noetic-ros-controllers
sudo apt install ros-noetic-rqt-controller-manager ros-noetic-rqt-joint-trajectory-controller
sudo apt install ros-noetic-ddynamic-reconfigure
sudo apt install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
```

### ROS tf tutorials
```
sudo apt install ros-noetic-ros-tutorials ros-noetic-geometry-tutorials ros-noetic-rviz ros-noetic-rosbash ros-noetic-rqt-tf-tree
```

### ROS joint state publisher GUI
```
sudo apt install ros-noetic-joint-state-publisher-gui
```

### URDF tools
```
sudo apt install liburdfdom-tools
```

### Catkin tools and Catkin_lint
```
sudo apt install python3-catkin-tools python3-osrf-pycommon catkin-lint
```

### Git
```
sudo apt install git
git config --global user.name "Your Name"
git config --global user.email "you@example.com"
git config --global push.default simple
```

### Documentation
```
sudo apt-get install doxygen
sudo apt-get install ros-noetic-rosdoc-lite
```
