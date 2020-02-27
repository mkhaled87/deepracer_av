# AWD DeepRacer Autonomous Vehicle

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
```


## Starting this ROS package
To start this package, we have a launch command to facilitate this:

```bash
roslaunch deepracer_av deepracer_av.launch
```

All of the AWS pre-installed ROS code load at boot time so starting this package is the only thing necessary once the installation is done.
