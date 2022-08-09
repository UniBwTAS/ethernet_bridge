#include "node.h"
#include <ethernet_msgs/utils.h>
#include <QHostAddress>


Node::Node(ros::NodeHandle& nh, ros::NodeHandle& private_nh) : nh_(nh), private_nh_(nh)
{
    /// Parameter
    // Topics
    private_nh.param<std::string>("topic_in", configuration_.topic_in, "bus_to_host");
    private_nh.param<std::string>("topic_out", configuration_.topic_out, "host_to_bus");

    // Redirection Rules
    private_nh.param<std::string>("redirect_address", configuration_.redirect_address, "");
    private_nh.param<int>("redirect_port", configuration_.redirect_port, 0);

    // Filter Rules
    private_nh.param<std::string>("filter_address", configuration_.filter_address, "");
    private_nh.param<int>("filter_port", configuration_.filter_port, 0);

    /// Subscribing & Publishing
    subscriber_ethernet_in_ = nh.subscribe(configuration_.topic_in, 100, &Node::rosCallback_ethernet_in, this);
    publisher_ethernet_out_ = nh.advertise<ethernet_msgs::Packet>(configuration_.topic_out, 100);

    /// Set redirection rule
    new_receiver_address_valid_ = false;
    if (configuration_.redirect_address.size() > 0)
    {
        QHostAddress address(QString::fromStdString(configuration_.redirect_address));
        if (!address.isNull())
        {
            bool ok;
            new_receiver_address_ = ethernet_msgs::arrayByNativeIp4(address.toIPv4Address(&ok));
            if (ok)
                new_receiver_address_valid_ = true;
        }
    }

    /// Set filter rule
    filter_address_valid_ = false;
    if (configuration_.filter_address.size() > 0)
    {
        QHostAddress address(QString::fromStdString(configuration_.filter_address));
        if (!address.isNull())
        {
            bool ok;
            filter_address_value_ = ethernet_msgs::arrayByNativeIp4(address.toIPv4Address(&ok));
            if (ok)
                filter_address_valid_ = true;
        }
    }

    /// Print config
    if (new_receiver_address_valid_)
        ROS_INFO_STREAM("Redirecting receiver IP to " << configuration_.redirect_address);
    if (configuration_.redirect_port)
        ROS_INFO_STREAM("Redirecting receiver port to " << configuration_.redirect_port);
    if (filter_address_valid_)
        ROS_INFO_STREAM("Only transmitting packets whose sender IP = " << configuration_.filter_address);
    if (configuration_.filter_port)
        ROS_INFO_STREAM("Only transmitting packets whose sender port = " << configuration_.filter_port);
    if (!new_receiver_address_valid_ && !configuration_.redirect_port && !filter_address_valid_ && !configuration_.filter_port)
        ROS_WARN("No redirection or filtering active");
}

Node::~Node()
{

}

void Node::rosCallback_ethernet_in(ethernet_msgs::Packet::Ptr msg)
{
    if (configuration_.redirect_port)
        msg->receiver_port = configuration_.redirect_port;

    if (new_receiver_address_valid_)
        msg->receiver_ip = new_receiver_address_;

    if (configuration_.filter_port)
        if (msg->sender_port != configuration_.filter_port)
            return;

    if (filter_address_valid_)
        if (msg->sender_ip != filter_address_value_)
            return;

    publisher_ethernet_out_.publish(msg);
}
