#include "node.h"
#include <ethernet_msgs/utils.h>
#include <QHostAddress>

Node::Node(ros::NodeHandle& nh, ros::NodeHandle& private_nh) : nh_(nh), private_nh_(nh)
{
    /// Parameter
    // Topics
    private_nh.param<std::string>("topic_in", configuration_.topic_in, "bus_to_host");

    // Sink Properties
    private_nh.param<std::string>("file_name", configuration_.file_name, "");
    private_nh.param<std::string>("packet_delimiter", configuration_.packet_delimiter, "");

    // Filter Rules
    private_nh.param<std::string>("filter_address", configuration_.filter_address, "");
    private_nh.param<int>("filter_port", configuration_.filter_port, 0);

    /// Subscribing & Publishing
    subscriber_ethernet_in_ = nh.subscribe(configuration_.topic_in, 100, &Node::rosCallback_ethernet_in, this);

    /// Create File Handle
    // Check for trailing newline
    if (!configuration_.file_name.empty() && configuration_.file_name.back() == '\n')
    {
        configuration_.file_name.pop_back();
        ROS_INFO("Removed trailing newline character in file_name");
    }

    // Open file
    file.open(configuration_.file_name, std::ofstream::out | std::ofstream::binary /*| std::ofstream::app*/);
    if (!file.is_open() || !file.good())
    {
        ROS_ERROR("Could not open file \"%s\" for writing. Aborting", configuration_.file_name.c_str());
        throw std::invalid_argument("Could not open file for writing.");
    }
    else
    {
        ROS_INFO_STREAM("Using file name: " << configuration_.file_name);
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
    if (filter_address_valid_)
        ROS_INFO_STREAM("Only storing packets whose sender IP = " << configuration_.filter_address);
    if (configuration_.filter_port)
        ROS_INFO_STREAM("Only storing packets whose sender port = " << configuration_.filter_port);
    if (!filter_address_valid_ && !configuration_.filter_port)
        ROS_INFO_STREAM("Storing packets from any IP and port");
}

Node::~Node()
{
    file.close();
}

void Node::rosCallback_ethernet_in(ethernet_msgs::Packet::Ptr msg)
{
    if (configuration_.filter_port)
        if (msg->sender_port != configuration_.filter_port)
            return;

    if (filter_address_valid_)
        if (msg->sender_ip != filter_address_value_)
            return;

    file.write(reinterpret_cast<char*>(msg->payload.data()), static_cast<long>(msg->payload.size()));

    if (configuration_.packet_delimiter.size() > 0)
        file << configuration_.packet_delimiter;

    file.flush();
}
