Pioneer_Model
=============

Slightly further adjusted Pioneer 3-DX model forked from https://github.com/SD-Robot-Vision/PioneerModel
that itself was a slightly adjusted pioneer model that was obtained from https://github.com/RafBerkvens/ua_ros_p3dx

(Forked from [https://github.com/cmcghan/PioneerModel](cmcghan/PioneerModel). AS4SR/PioneerModel is the most up-to-date fork in this series.)

Quick Description
-----------------

From prior README.md file: "A ROS/Gazebo Pioneer 3DX model."


Package Dependencies
--------------------

To run this successfully, we require:
* ROS Kinetic and Gazebo 5+

Specific dependencies:
* PioneerModel/p3dx_control requires controller_manager to compile (controller_manager_tests auto-installs this)
* gazebo_ros_pkgs is needed to provide the necessary interfaces to integrate simulated robots with ROS
  * gazebo_ros_control is a subpackage of this that provides some basic controllers / control interfaces (plugins) that simulated robots can use
* gazebo_ros_control needs transmission_interface from ros_control

To install ROS kinetic and set up a catkin workspace, try:
```
git clone https://github.com/AS4SR/vagrant-rss.git
./vagrant-rss/single_installers/quick_install_ros_gazebo_and_catkin_ws.sh kinetic $USER
```

Install additional ROS kinetic dependencies:
```
$ sudo apt-get install ros-kinetic-controller-manager-tests ros-kinetic-ros-controllers
$ sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-ros-control
```

Optional installation, if you want to test keyboard control:
```
$ cd <catkin_ws>/src
$ git clone https://github.com/SD-Robot-Vision/p3dx_mover.git
$ cd ..
$ catkin_make
```


Compilation for Use
-------------------

To download and compile this github repository (assuming you've already installed `ros-kinetic-desktop-full` but don't have a catkin workspace set up yet):
```
$ mkdir -p <catkin_ws>/src
$ cd <catkin_ws>/src
$ git clone https://github.com/AS4SR/PioneerModel.git
$ cd ..
$ catkin_make
```

...Alternately, to install this github repository for rss-rse use:
```
$ mkdkir -p <catkin_ws>/src/rss-rse/extern
$ cd <catkin_ws>/src/rss-rse/extern
$ git clone https://github.com/AS4SR/PioneerModel.git
$ cd ../../..
$ catkin_make
```

NOTE:
* As of 2017-06-21, updates have been made to make these packages work properly with the newer versions of Gazebo (5+) and ros-kinetic-xacro that may break compatibility with older versions of ROS and Gazebo. However, if you'd still like to try using this package with an older version of ROS, you can try the following:

For ROS indigo:
```
$ cd <catkin_ws>/src
$ sudo apt-get install ros-indigo-controller-manager-tests ros-indigo-ros-controllers
$ sudo apt-get install ros-indigo-gazebo-ros-control
```

For ROS jade:
```
$ sudo apt-get install ros-jade-controller-manager-tests ros-jade-ros-controllers
$ sudo apt-get install ros-jade-gazebo-ros-pkgs ros-jade-ros-control
```


Testing
-------

Try running the 0$ and 1$ commands here in separate terminals:
```
0$ roscore
1$ roslaunch p3dx_gazebo_mod gazebo.launch 
```
to show the p3dx rover in a Willow Garage setting.

If you also installed the optional keyboard controller, try running this in a third terminal:
```
2$ rosrun p3dx_mover mover.py 
```
then use the arrow keys to control the robot motion, 'q' to quit.

References:  
http://wiki.lofarolabs.com/index.php/Moving_The_Pioneer_3-DX_In_Gazebo


License
=======

Original license is GPLv3, see LICENSE file in the repository.

Modifications from SD-Robot-Vision/PioneerModel fork are Copyright (c) 2016, by California Institute of Technology, also under GPLv3,
and Copyright (c) 2017 by University of Cincinnati, also under GPLv3.

Modifications were performed by Catharine McGhan.

Modifications include:
- updates to README.md file
- changing the package names in all subdirectories to *_mod (for compatibility in case original branch is downloaded)
- changes to directory names to make them match the package names in CMakeLists.txt and package.xml
- updates to maintainer email addresses in package.xml files
- removing .project and meow.txt files
- fixing internal naming inside robot model (base_link_*_wheel_joint instead of base_*_wheel_joint)
- adding <hardwareInterface> element to <joint> element inside robot model
- adding default parameters for DiffDrive
- exposing (x,y,z,R,P,Y) parameters (starting position and orientation of p3dx rover) in gazebo.launch
- fixing a typo in the urdf_spawner call in the gazebo.launch file (" was in the wrong place)
- fixes to other internals to remove error messages (legacyMode and leftJoint rightJoint in gazebo.launch; odometrySource)
- updates to support and use the newer `xacro -i` (with "inorder" processing mode) instead of deprecated `xacro.py`
- fix to material-color in model files (removal of 'name=' property from all 'visual' tags)

References:  
- http://answers.ros.org/question/186681/no-valid-hardware-interface-element-found-in-joint/  
- http://answers.ros.org/question/40627/how-do-i-set-the-inital-pose-of-a-robot-in-gazebo/  
- http://gazebosim.org/tutorials/?tut=ros_roslaunch
- http://docs.ros.org/kinetic/api/gazebo_plugins/html/gazebo__ros__diff__drive_8cpp_source.html
- http://wiki.ros.org/diff_drive_controller
- http://wiki.ros.org/xacro#Processing_Order
- https://bitbucket.org/osrf/sdformat/issues/132/parser-does-not-handle-urdf-material

Contact
=======

If you have any questions regarding the contents of this repository, please email Catharine McGhan at <cat.mcghan@uc.edu>.

-EOF-
