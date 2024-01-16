# scale_truck_control_gui

GUI controller for [SuneelFreimuth/scale_truck_control](https://github.com/SuneelFreimuth/scale_truck_control/).

## Setup

scale_truck_control_gui depends on:
* ROS Melodic
* OpenCV 4.4.0
* Qt 5

### Step 1: Install ROS Melodic

Add the ROS repositories to `/etc/apt/sources.list.d/`:
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

Add apt keys:
```
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

Install ROS Melodic:
```
sudo apt install ros-melodic-desktop-full
```

ROS Melodic provides `setup.bash`, a script which configures the current shell's environment. Do one or both of the following:
```
# To update the environment on startup for every future shell session:
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc


# To update the environment for the current session:
source /opt/ros/melodic/setup.bash
```

### Step 2: Install OpenCV 4.4.0

TODO

### Step 3: Install Qt 5

TODO

### Step 4: Set Up a Catkin Workspace

Create a Catkin workspace:
```
mkdir -p ~/controller_ws/src
cd ~/controller_ws/
catkin_make
```

Enter the source space and clone this repository:
```
cd src
git clone https://github.com/SuneelFreimuth/scale_truck_control_gui/
```

### Step 5: Run the GUI

Build the controller:
```
cd ~/controller_ws
catkin_make
```

And finally run the ROS node:
```
rosrun scale_truck_control_gui gui_controller
```
