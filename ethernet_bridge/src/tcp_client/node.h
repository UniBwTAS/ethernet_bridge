#pragma once

#include <ros/ros.h>
#include <ethernet_msgs/Packet.h>
#include <QObject>
#include <QTimer>

class QTcpSocket;

class Node : public QObject
{
    Q_OBJECT

public:
    Node(ros::NodeHandle& node_handle);
    ~Node();

private:
    // Reference to ROS Handle
    ros::NodeHandle& ros_handle_;

    // ROS Interfaces
    ros::Subscriber subscriber_ethernet_;
    ros::Publisher 	publisher_ethernet_packet_;
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
        std::string ethernet_peerAddress;
        int         ethernet_peerPort;
        int         ethernet_bufferSize;
        int         ethernet_reconnectInterval; // [ms]

    }   configuration_;

    // ethernet
    QTcpSocket* socket_;

    // timer
    QTimer timer_;

private slots:
    void slotEthernetNewData();
    void slotEthernetConnected();
    void slotEthernetDisconnected();
    void slotEthernetError(int error_code);
    void slotTimer();

    // cache for runtime optimization
private:
    ethernet_msgs::Packet packet;
};
