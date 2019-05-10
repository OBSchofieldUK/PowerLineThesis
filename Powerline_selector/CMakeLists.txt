cmake_minimum_required(VERSION 2.8.3)
project(powerline_selector)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED 
  roscpp
  std_msgs 
  inspec_msg
  inspec_lib
  rw_lib
  )

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    std_msgs
    inspec_msg
    inspec_lib
    rw_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/main.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rocketbot.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
