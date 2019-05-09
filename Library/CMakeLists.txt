cmake_minimum_required(VERSION 2.8.3)
project(inspec_lib)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED 
  roscpp
  std_msgs 
  inspec_msg
  roslib
  rw_lib
  )

find_package(OpenCV REQUIRED)
find_package(RapidJSON REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    std_msgs
    inspec_msg
    roslib
    rw_lib
  DEPENDS 
    OpenCV
)

###########
## Build ##
###########


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/inspec_lib/Math.cpp
  src/converters/RosConverters.cpp
  src/converters/CoordinateConverters.cpp
  src/converters/OtherConverters.cpp 
  src/settings/ReadSettings.cpp
)
add_dependencies(${PROJECT_NAME} inspec_msg_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
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
