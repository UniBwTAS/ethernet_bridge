#pragma once

#include <ros/ros.h>
#include <ethernet_msgs/Packets.h>
#include <QObject>
#include <QTimer>

class QUdpSocket;

class Node : public QObject
{
    Q_OBJECT

public:
    Node(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
    ~Node();

private:
    // Reference to ROS Handle
    ros::NodeHandle& nh_;
    ros::NodeHandle& private_nh_;

    // ROS Interfaces
    ros::Subscriber subscriber_ethernet_;
    ros::Publisher 	publisher_ethernet_packets_;
    ros::Publisher 	publisher_ethernet_event_;

private:
    // ROS data reception callbacks
    void rosCallback_ethernet(const ethernet_msgs::Packet::ConstPtr &msg);

private:
    // configuration
    struct
    {
        std::string topic_busToHost;
        std::string topic_hostToBus;
        std::string topic_event;
        std::string frame;
        std::string ethernet_bindAddress;
        int         ethernet_bindPort;
        int         trigger_numberOfPackets;
        int         trigger_maximumPacketAge;
        int         trigger_maximumIdleTime;
    }   configuration_;

    // ethernet
    QUdpSocket* socket_;

    // bundler buffer
    ethernet_msgs::Packets bundler_buffer_;
    QTimer bundler_age_timer_;
    QTimer bundler_idle_timer_;

private slots:
    void slotEthernetNewData();
    void slotEthernetConnected();
    void slotEthernetDisconnected();
    void slotEthernetError(int error_code);

    void slotBundlerAgeExceeded();
    void slotBundlerIdleExceeded();

private:
    ethernet_msgs::Packet packet;
    void transmitBuffer();
};
