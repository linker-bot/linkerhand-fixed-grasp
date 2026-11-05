cmake_minimum_required(VERSION 3.8)
project(HandTCPCommunication)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(catkin REQUIRED COMPONENTS
  roscpp roslib std_msgs sensor_msgs geometry_msgs
  tf tf2 tf2_ros tf2_geometry_msgs serial message_generation)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)

add_message_files(FILES TCPRecvSensorData.msg OrderDetectResult.msg)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES TCPCommunication_Hand
  CATKIN_DEPENDS roscpp roslib std_msgs sensor_msgs geometry_msgs
    tf tf2 tf2_ros tf2_geometry_msgs serial message_runtime)
include_directories(include handeyesolver/include ${catkin_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS})
link_directories(${JSONCPP_LIBRARY_DIRS})

set(RM_SERVICE_LIBRARY "" CACHE FILEPATH "RealMan SDK shared library for the target architecture")
if(NOT RM_SERVICE_LIBRARY)
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
    set(RM_SERVICE_LIBRARY "${CMAKE_CURRENT_SOURCE_DIR}/lib/libRM_Service.so")
  else()
    message(FATAL_ERROR "Set RM_SERVICE_LIBRARY to a RealMan SDK library for ${CMAKE_SYSTEM_PROCESSOR}; the bundled SDK is ARM64")
  endif()
endif()
if(NOT EXISTS "${RM_SERVICE_LIBRARY}")
  message(FATAL_ERROR "RM_SERVICE_LIBRARY does not exist: ${RM_SERVICE_LIBRARY}")
endif()
add_library(RM_Service SHARED IMPORTED)
set_target_properties(RM_Service PROPERTIES IMPORTED_LOCATION "${RM_SERVICE_LIBRARY}")

add_subdirectory(handeyesolver)
add_library(TCPCommunication_Hand
  src/TCPCommunication_Hand.cpp src/TcpStream.cpp src/CoordinateTransform.cpp src/HandEyeCalibration.cpp)
add_dependencies(TCPCommunication_Hand ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(TCPCommunication_Hand PUBLIC handeyesolver Eigen3::Eigen
  ${catkin_LIBRARIES} ${JSONCPP_LIBRARIES} RM_Service)
add_executable(HandCtrl_node src/HandCtrl_node.cpp)
add_dependencies(HandCtrl_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(HandCtrl_node TCPCommunication_Hand)
set_target_properties(HandCtrl_node PROPERTIES INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES target_pose.json tools_Tcp.json CalibrateMatrix_EyeInHand.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY include/ handeyesolver/include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
install(TARGETS TCPCommunication_Hand handeyesolver HandCtrl_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES "${RM_SERVICE_LIBRARY}" DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} RENAME libRM_Service.so)
install(FILES "${RM_SERVICE_LIBRARY}" DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} RENAME libRM_Service.so.1)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(grasp_regression test/grasp_regression.test test/grasp_regression.cpp)
  if(TARGET grasp_regression)
    target_link_libraries(grasp_regression TCPCommunication_Hand)
  endif()
  find_package(Threads REQUIRED)
  catkin_add_gtest(tcp_stream_regression test/tcp_stream_regression.cpp src/TcpStream.cpp)
  if(TARGET tcp_stream_regression)
    target_link_libraries(tcp_stream_regression Threads::Threads)
  endif()
  add_rostest_gtest(tcp_recovery_regression test/tcp_recovery_regression.test test/tcp_recovery_regression.cpp)
  if(TARGET tcp_recovery_regression)
    target_link_libraries(tcp_recovery_regression TCPCommunication_Hand)
  endif()
endif()
