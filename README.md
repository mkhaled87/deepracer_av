# AWD DeepRacer Autonomous Vehicle [under development]

This repository will add some ROS nodes to control the AWS DeepRacer autonomously.

Pre-installed AWS startup files load ROS components at boot time.  This setup does not mess with the pre-installed components, it only adds additional ROS nodes.  

## Installation

First, login to the DeepRacer through the ubuntu login form (or open an SSH session if you enabled SSH to the DeepRacer). In case you did not source the required parameters, run the following command:

```bash
source /opt/aws/deepracer/setup.bash
```

Also, if you have not already created a catkin workspace you can follow the [instructions](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) to create one. We assume you have the catkin workspace in your home directory.
Now, navigate to it and clone this repo:

```bash
cd ~/catkin_ws/src/
git clone https://github.com/mkhaled87/deepracer_av.git
```

Make the python files executable and editable:

```bash
chmod +rwx ~/catkin_ws/src/deepracer_av/scripts/*.py
```

Finally, build the package:

```bash
chmod +rwx ~/catkin_ws/
catkin_make
catkin_make install
```

All the above steps (after cloning) are automated in the script file **install.sh**. Again, the code in the automation script assumes that catkin workspace is located in the home directory.

## Starting this ROS package

To start this package, we have a launch command to facilitate this:

```bash
roslaunch deepracer_av deepracer_av.launch
```

All of the AWS pre-installed ROS code load at boot time so starting this package is the only thing necessary once the installation is done.


## Working/Testing locally

In case you dont have access to AWS DeepRacer or like to test locally, and assuming you have access to a machine
with the same version of ROS installed, we created a node to emulated the camera and load recorded video streams
for the sake of testing. Run the following command to start the camera emulator.

```bash
rosrun deepracer_av camera_emulator.py
```

The **camera_emulator.py** loads a video file from those in the folder [media/mp4/](/media/mp4/).
