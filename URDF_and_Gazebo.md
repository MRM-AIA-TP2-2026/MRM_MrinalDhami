## Skills learned

1:- How to model robots using a URDF file
2:- How to use xacro files for better efficiency
3:-How to use Gazebo and Rviz to visualize your model
4:-How to use ROS and Gazebo plugins to program your model beahviour


## Creating Launch files and URDF(Xacro) 3D model in Gazebo using ROS2

### Firstly, make sure you have the correct Linux and ros version.
This can be done using the following code
`cat /etc/os-release`
for your os, source your bash file and then
`printenv ROS DISTRO`

Then we create new package, but first make sure I have all the dependencies resolved by installing the following
    sudo apt-get update
    sudo apt-get install gedit
    sudo apt install ros-humble-joint-state-publisher
    sudo apt install ros-humble-joint-state-publisher-gui
    sudo apt install ros-humble-xacro

Now install `gazebo` tools using
    sudo apt install ros-humble-gazebo-ros-pkgs
    sudo apt install ros-humble-ros-core ros-humble-geometry2
