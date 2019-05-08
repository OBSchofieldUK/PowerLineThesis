cmake_minimum_required(VERSION 2.8.3)
project(rw_lib)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED 
  roscpp
  )

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

find_package(Eigen3 REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
)

###########
## Build ##
###########


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/Exception.cpp 
  src/Extension.cpp 
  src/ExtensionRegistry.cpp
  src/IOUtil.cpp
  src/LinearAlgebra.cpp
  src/Log.cpp
  src/LogStreamWriter.cpp
  src/LogWriter.cpp
  src/Math.cpp
  src/Message.cpp
  src/PropertyBase.cpp
  src/PropertyMap.cpp
  src/PropertyType.cpp
  src/Quaternion.cpp
  src/Random.cpp
  src/RobWork.cpp
  src/Rotation3D.cpp
  src/Rotation3DVector.cpp
  src/RPY.cpp
  src/StringUtil.cpp
  src/Timer.cpp 
  src/TimerUtil.cpp 
  src/Transform3D.cpp
  src/Vector3D.cpp
)
#add_dependencies(${PROJECT_NAME} inspec_msg_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
   Eigen3::Eigen
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
