cmake_minimum_required(VERSION 2.8.3)
project(square_demo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
)

catkin_package()


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(square src/square.cpp)
add_executable(CLsquare src/CLsquare.cpp)
add_executable(Odom_sub src/Odom_sub.cpp)
target_link_libraries(square ${catkin_LIBRARIES})
target_link_libraries(CLsquare ${catkin_LIBRARIES})
target_link_libraries(Odom_sub ${catkin_LIBRARIES})
add_dependencies(square square_demo_generate_messages_cpp )
add_dependencies(CLsquare square_demo_generate_messages_cpp )
add_dependencies(Odom_sub nav_msgs_generate_messages_cpp)
