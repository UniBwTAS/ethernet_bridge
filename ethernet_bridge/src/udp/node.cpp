#include "node.h"
#include <QUdpSocket>
#include <QNetworkDatagram>
#include <ethernet_msgs/msg/event.hpp>
#include <ethernet_msgs/msg/event_type.hpp>
#include <ethernet_msgs/msg/protocol_type.hpp>
#include <ethernet_msgs/utils.hpp>


Node::Node(const std::string& name) : rclcpp::Node(name)
{
    /// Parameter
    // Frame
    this->declare_parameter<std::string>("frame", "");
    this->get_parameter("frame", configuration_.frame);

    // Ethernet connection
    this->declare_parameter<std::string>("ethernet_bindAddress", "0.0.0.0");
    this->get_parameter("ethernet_bindAddress", configuration_.ethernet_bindAddress);
    this->declare_parameter<int>("ethernet_bindPort", 55555);
    this->get_parameter("ethernet_bindPort", configuration_.ethernet_bindPort);

    /// Subscribing & Publishing
    subscriber_ethernet_ = this->create_subscription<ethernet_msgs::msg::Packet>("host_to_bus", rclcpp::SensorDataQoS().keep_last(10000), std::bind(&Node::rosCallback_ethernet, this, std::placeholders::_1));
    publisher_ethernet_packet_ = this->create_publisher<ethernet_msgs::msg::Packet>("bus_to_host", 10000);
    publisher_ethernet_event_ = this->create_publisher<ethernet_msgs::msg::Event>("event", rclcpp::QoS(1).transient_local());

    /// Initialize socket
    socket_ = new QUdpSocket(this);
    connect(socket_, SIGNAL(readyRead()), this, SLOT(slotEthernetNewData()));
    connect(socket_, SIGNAL(connected()), this, SLOT(slotEthernetConnected()));
    connect(socket_, SIGNAL(disconnected()), this, SLOT(slotEthernetDisconnected()));
#if (QT_VERSION >= QT_VERSION_CHECK(5, 15, 0))
    connect(socket_, SIGNAL(errorOccurred(QAbstractSocket::SocketError)), this, SLOT(slotEthernetError(QAbstractSocket::SocketError)));
#endif

    // Publish unconnected state
    slotEthernetDisconnected();

    // Don't share port (Qt-Socket)
//  bool success = socket_->bind(QHostAddress(QString::fromStdString(configuration_.ethernet_bindAddress)), configuration_.ethernet_bindPort, QAbstractSocket::DontShareAddress);

    // Share port (Qt-Socket)
    bool success = socket_->bind(QHostAddress(QString::fromStdString(configuration_.ethernet_bindAddress)), configuration_.ethernet_bindPort, QAbstractSocket::ShareAddress | QUdpSocket::ReuseAddressHint);

    // Share port (native Linux socket)
//  #include <sys/socket.h>
//  int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
//  int optval = 1;
//  setsockopt(sockfd, SOL_SOCKET, SO_REUSEPORT, (void *) &optval, sizeof(optval));
//  socket_->setSocketDescriptor(sockfd, QUdpSocket::UnconnectedState);
//  bool success = socket_->bind(QHostAddress(QString::fromStdString(configuration_.ethernet_bindAddress)), configuration_.ethernet_bindPort, QAbstractSocket::ShareAddress | QUdpSocket::ReuseAddressHint);

    RCLCPP_INFO(get_logger(), "Binding to %s:%u -> %s", QHostAddress(QString::fromStdString(configuration_.ethernet_bindAddress)).toString().toLatin1().data(), configuration_.ethernet_bindPort, success?"ok":"failed");
}

Node::~Node()
{
    delete socket_;
}

void Node::rosCallback_ethernet(const ethernet_msgs::msg::Packet::ConstSharedPtr &msg)
{
    QNetworkDatagram datagram(QByteArray(reinterpret_cast<const char*>(msg->payload.data()), msg->payload.size()), QHostAddress(ethernet_msgs::nativeIp4ByArray(msg->receiver_ip)), msg->receiver_port);
    if (ethernet_msgs::nativeIp4ByArray(msg->sender_ip) != 0)
        datagram.setSender(QHostAddress(ethernet_msgs::nativeIp4ByArray(msg->sender_ip)), msg->sender_port);

    socket_->writeDatagram(datagram);
}

void Node::slotEthernetNewData()
{
    while (socket_->hasPendingDatagrams())
    {
        QNetworkDatagram datagram = socket_->receiveDatagram();
        //  ethernet_msgs::Packet packet; // moved to private member for optimization

        packet.header.stamp = this->get_clock()->now();
        packet.header.frame_id = configuration_.frame;

        packet.sender_ip = ethernet_msgs::arrayByNativeIp4(datagram.senderAddress().toIPv4Address());
        packet.sender_port = datagram.senderPort();
        packet.receiver_ip = ethernet_msgs::arrayByNativeIp4(datagram.destinationAddress().toIPv4Address());
        packet.receiver_port = datagram.destinationPort();

        packet.payload.clear();
        packet.payload.reserve(datagram.data().count());
        std::copy(datagram.data().constBegin(), datagram.data().constEnd(), std::back_inserter(packet.payload));

        publisher_ethernet_packet_->publish(packet);
    }
}

// Note: UDP sockets are usually not "connected" or "disconnected"
void Node::slotEthernetConnected()
{
    ethernet_msgs::msg::Event event;

    event.header.stamp = this->get_clock()->now();
    event.header.frame_id = configuration_.frame;

    event.type = ethernet_msgs::msg::EventType::CONNECTED;

    publisher_ethernet_event_->publish(event);

    RCLCPP_INFO(get_logger(), "Connected.");
}

void Node::slotEthernetDisconnected()
{
    ethernet_msgs::msg::Event event;

    event.header.stamp = this->get_clock()->now();
    event.header.frame_id = configuration_.frame;

    event.type = ethernet_msgs::msg::EventType::DISCONNECTED;

    publisher_ethernet_event_->publish(event);

    RCLCPP_INFO(get_logger(), "Disconnected.");
}

void Node::slotEthernetError(QAbstractSocket::SocketError error_code)
{
    ethernet_msgs::msg::Event event;

    event.header.stamp = this->get_clock()->now();
    event.header.frame_id = configuration_.frame;

    event.type = ethernet_msgs::msg::EventType::SOCKETERROR;
    event.value = error_code;

    publisher_ethernet_event_->publish(event);

    RCLCPP_WARN(get_logger(), "Connection error occured, socket error code: %i", error_code);
}
