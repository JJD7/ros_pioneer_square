
## Setting up
Make sure you already have ROS kinetic and gazebo 7+ installed. Install the required dependencies. 

`$ sudo apt-get install ros-kinetic-controller-manager-tests ros-kinetic-ros-controllers ros-kinetic-gazebo-ros-pkgs ros-kinetic-ros-control`

Clone the source code for HW1

`$ git clone https://github.com/VT-robot-motion-planning-course/F2018-HW1.git`

`$ git submodule update --init --recursive`

Change to the catkin_ws

`$ cd catkin_ws`

Compile the demo code

`$ catkin_make`

Source the setup.bash file. *Remember to do this in everytime you open a new terminal. Else, you will likely get an error that says some package not found.*

`$ source devel/setup.bash`


## Running the code
Run the Gazebo simulator launch file.

`$ roslaunch p3dx_gazebo gazebo.launch`

Run the node that controls the speed of the robot (by publishing on the cmd\_vel topic). You can run either the C++ version:

`$ rosrun square_demo square`

or the Python version:

`$ rosrun square_demo square.py`

or the MATLAB version by opening and running the square.m file in MATLAB. You'll have to install the MATLAB Robotics Systems Toolbox.
