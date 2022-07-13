cmake_minimum_required(VERSION 3.8)
project(ethernet_bridge)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ethernet_msgs REQUIRED)
find_package(Qt5 COMPONENTS Core Network REQUIRED)
find_package(libros2qt REQUIRED)

set(CMAKE_AUTOMOC ON)

## UDP Bridge
add_executable(udp src/udp/main.cpp src/udp/node.cpp)
target_link_libraries(udp Qt5::Core Qt5::Network)
ament_target_dependencies(udp std_msgs rclcpp ethernet_msgs libros2qt)

## TCP Client Bridge
# add_executable(tcp_client src/tcp_client/main.cpp src/tcp_client/node.cpp)
# target_link_libraries(tcp_client ${catkin_LIBRARIES} Qt5::Core Qt5::Network)

## UDP Bundler Bridge
# add_executable(udp_bundler src/udp_bundler/main.cpp src/udp_bundler/node.cpp)
# target_link_libraries(udp_bundler ${catkin_LIBRARIES} Qt5::Core Qt5::Network)

## Redirector
# add_executable(redirector src/redirector/main.cpp src/redirector/node.cpp)
# target_link_libraries(redirector ${catkin_LIBRARIES} Qt5::Core Qt5::Network)

# install executables
install(TARGETS
    udp
    DESTINATION lib/${PROJECT_NAME})

# install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

# install(DIRECTORY launch
#     DESTINATION share/${PROJECT_NAME})

# Export modern CMake targets
# ament_export_targets(export_${PROJECT_NAME})

ament_export_dependencies(
    std_msgs
    rclcpp
    ethernet_msgs
    libros2qt)

ament_package()
