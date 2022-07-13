cmake_minimum_required(VERSION 3.8)
project(ethernet_msgs)

find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    msg/Packet.msg
    msg/GenericPacket.msg
    msg/ProtocolType.msg
    msg/Event.msg
    msg/EventType.msg
    msg/Packets.msg
    DEPENDENCIES std_msgs)

install(DIRECTORY include/
    DESTINATION include/${PROJECT_NAME})

ament_package()