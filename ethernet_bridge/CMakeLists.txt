cmake_minimum_required(VERSION 3.0.2)
project(ethernet_bridge)

#add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11" CACHE STRING "" FORCE)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  ethernet_msgs
  librosqt
)

find_package(Qt5 5.9 COMPONENTS
    Core
    Network
    REQUIRED
)
set(CMAKE_AUTOMOC ON)

catkin_package(
  CATKIN_DEPENDS std_msgs ethernet_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## UDP Bridge
add_executable(udp src/udp/main.cpp src/udp/node.cpp)
target_link_libraries(udp ${catkin_LIBRARIES} Qt5::Core Qt5::Network)

## TCP Client Bridge
add_executable(tcp_client src/tcp_client/main.cpp src/tcp_client/node.cpp)
target_link_libraries(tcp_client ${catkin_LIBRARIES} Qt5::Core Qt5::Network)

## UDP Bundler Bridge
add_executable(udp_bundler src/udp_bundler/main.cpp src/udp_bundler/node.cpp)
target_link_libraries(udp_bundler ${catkin_LIBRARIES} Qt5::Core Qt5::Network)

## Redirector
add_executable(redirector src/redirector/main.cpp src/redirector/node.cpp)
target_link_libraries(redirector ${catkin_LIBRARIES} Qt5::Core Qt5::Network)
