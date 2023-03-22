#pragma once

#include <ros/ros.h>
#include <ethernet_msgs/Packet.h>

#include <iostream>
#include <fstream>


class Node
{

public:
    Node(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
    ~Node();

private:
    // Reference to ROS Handle
    ros::NodeHandle& nh_;
    ros::NodeHandle& private_nh_;

    // ROS Interfaces
    ros::Subscriber subscriber_ethernet_in_;

private:
    // ROS data reception callbacks
    void rosCallback_ethernet_in(ethernet_msgs::Packet::Ptr msg);

private:
    // configuration
    struct
    {
        std::string topic_in;
        std::string file_name;
        std::string packet_delimiter;
        std::string filter_address;
        int         filter_port;
    }   configuration_;

    // file
    std::ofstream file;

    // filter rule
    ethernet_msgs::Packet::_sender_ip_type filter_address_value_;
    bool filter_address_valid_{false};
};
