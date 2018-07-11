cmake_minimum_required(VERSION 2.8.3)
project(galileo_serial_server)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  message_generation
)


 add_message_files(
   FILES
   GalileoNativeCmds.msg
   GalileoStatus.msg
 )


 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs tf
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

set(BOOST_LIBS thread date_time system)
find_package(Boost COMPONENTS ${BOOST_LIBS} REQUIRED)
find_package(Threads REQUIRED)


add_executable(galileo_serial_server_node src/galileo_serial_server_node.cpp src/galileo_serial_server.cpp src/AsyncSerial.cpp)

add_dependencies(galileo_serial_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(galileo_serial_server_node
 ${catkin_LIBRARIES}
)

install(TARGETS galileo_serial_server_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

