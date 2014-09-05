takktile-ros
============

ROS drivers for the TakkTile tactile array

## Installation

### Basic Requirements

You must install TakkTile TakkFast USB drivers (see https://github.com/TakkTile/TakkTile-usb)

This will install `TakkTile.py` which is a dependency (currently included as a symlink -- need to fix this)


### Repository Installation

Go to your ROS working directory. e.g.
```
cd ~/ros_ws/src
``` 

Clone the repository
```
git clone --recursive https://github.com/fsuarez6/takktile_ros.git -b hydro-devel
``` 

Set USB permissions
```
cd takktile_ros
sudo cp 71-takktile.rules /etc/udev/rules.d/
``` 
 
Install any missing dependencies using rosdep:
```
rosdep update
rosdep install --from-paths . --ignore-src --rosdistro <groovy | hydro | indigo>
``` 

Now compile your ROS workspace. e.g.
```
cd ~/ros_ws && catkin_make
``` 

### Testing the Installation
```
rosrun takktile_ros takktile_node.py
``` 
Check the output using `rqt_plot` (in another terminal while `takktile_node.py` is running)
```
rqt_plot /takktile/calibrated/pressure[0]:pressure[1]:pressure[2]:pressure[3]:pressure[4]
``` 
