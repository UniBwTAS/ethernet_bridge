#pragma once

#include <ros/ros.h>
#include <ethernet_msgs/Packet.h>

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
    ros::Publisher 	publisher_ethernet_out_;

private:
    // ROS data reception callbacks
    void rosCallback_ethernet_in(ethernet_msgs::Packet::Ptr msg);

private:
    // configuration
    struct
    {
        std::string topic_in;
        std::string topic_out;
        std::string redirect_address;
        int redirect_port;
    }   configuration_;

    // redirection rule
    ethernet_msgs::Packet::_receiver_ip_type new_receiver_address_;
    bool new_receiver_address_valid_{false};
};
