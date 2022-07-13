#pragma once

#include <rclcpp/rclcpp.hpp>
#include <QAbstractSocket>
#include <ethernet_msgs/msg/packet.hpp>
#include <ethernet_msgs/msg/event.hpp>
#include <QObject>

class QUdpSocket;

class Node : public QObject, public rclcpp::Node
{
    Q_OBJECT

public:
    Node(const std::string& name);
    ~Node();

private:
    // ROS Interfaces
    rclcpp::Subscription<ethernet_msgs::msg::Packet>::SharedPtr subscriber_ethernet_;
    rclcpp::Publisher<ethernet_msgs::msg::Packet>::SharedPtr publisher_ethernet_packet_;
    rclcpp::Publisher<ethernet_msgs::msg::Event>::SharedPtr publisher_ethernet_event_;

private:
    // ROS data reception callbacks
    void rosCallback_ethernet(const ethernet_msgs::msg::Packet::ConstSharedPtr &msg);

private:
    // configuration
    struct
    {
        std::string frame;
        std::string ethernet_bindAddress;
        int         ethernet_bindPort;

    }   configuration_;

    // ethernet
    QUdpSocket* socket_;

private slots:
    void slotEthernetNewData();
    void slotEthernetConnected();
    void slotEthernetDisconnected();
    void slotEthernetError(QAbstractSocket::SocketError error_code);

    // cache for runtime optimization
private:
    ethernet_msgs::msg::Packet packet;
};
