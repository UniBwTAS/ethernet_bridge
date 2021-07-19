cmake_minimum_required(VERSION 3.0.2)
project(ethernet_msgs)

#add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11" CACHE STRING "" FORCE)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

add_message_files(
  FILES
  Packet.msg
  GenericPacket.msg
  ProtocolType.msg
  Event.msg
  EventType.msg
  Packets.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
