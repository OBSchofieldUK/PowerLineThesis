cmake_minimum_required(VERSION 2.8.3)
project(leddar_lib)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED 
  roscpp
  )


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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

  ${catkin_INCLUDE_DIRS})

## Declare a C++ library
add_library(${PROJECT_NAME}

  src/LdBitFieldProperty.cpp
  src/LdBoolProperty.cpp
  src/LdBufferProperty.cpp
  src/LdCarrierEnhancedModbus.cpp
  src/LdConnection.cpp
  src/LdConnectionFactory.cpp
  src/LdConnectionInfo.cpp
  src/LdConnectionInfoEthernet.cpp
  src/LdConnectionInfoModbus.cpp
  src/LdConnectionInfoSpi.cpp
  src/LdConnectionInfoUsb.cpp
  src/LdConnectionUniversal.cpp
  src/LdConnectionUniversalModbus.cpp
  src/LdConnectionUniversalSpi.cpp
  src/LdDevice.cpp
  src/LdDeviceFactory.cpp
  src/LdDoubleBuffer.cpp
  src/LdEnumProperty.cpp
  src/LdEthernet.cpp
  src/LdFloatProperty.cpp
  src/LdIntegerProperty.cpp
  src/LdInterfaceSpi.cpp
  src/LdInterfaceUsb.cpp
  src/LdLibModbusSerial.cpp
  src/LdLibUsb.cpp
  src/LdObject.cpp
  src/LdPropertiesContainer.cpp
  src/LdProperty.cpp
  src/LdProtocolLeddarTech.cpp
  src/LdProtocolLeddartechUSB.cpp
  src/LdResultEchoes.cpp
  src/LdResultProvider.cpp
  src/LdResultStates.cpp
  src/LdSensor.cpp
  src/LdSensorM16.cpp
  src/LdSensorM16Laser.cpp
  src/LdSensorM16Modbus.cpp
  src/LdSensorOneModbus.cpp
  src/LdSensorVu8.cpp
  src/LdSensorVu8Modbus.cpp
  src/LdSensorVu.cpp
  src/LdSpiBCM2835.cpp
  src/LdSpiFTDI.cpp
  src/LdTextProperty.cpp
  src/LtCRCUtils.cpp
  src/LtFileUtils.cpp
  src/LtKeyboardUtils.cpp
  src/LtStringUtils.cpp
  src/LtSystemUtils.cpp
  src/LtTimeUtils.cpp
  src/modbus.c
  src/modbus-data.c
  src/modbus-LT.c
  src/modbus-rtu.c
  src/modbus-tcp.c

)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
   usb-1.0
   dl
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
