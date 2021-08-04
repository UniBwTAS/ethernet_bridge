#include "node.h"
#include <QTcpSocket>
#include <QHostAddress>
#include <ethernet_msgs/Event.h>
#include <ethernet_msgs/EventType.h>
#include <ethernet_msgs/ProtocolType.h>
#include <ethernet_msgs/utils.h>


Node::Node(ros::NodeHandle& nh, ros::NodeHandle& private_nh) : nh_(nh), private_nh_(nh)
{
    /// Parameter
    // Topics
    private_nh.param<std::string>("topic_busToHost", configuration_.topic_busToHost, "bus_to_host");
    private_nh.param<std::string>("topic_hostToBus", configuration_.topic_hostToBus, "host_to_bus");
    private_nh.param<std::string>("topic_event", configuration_.topic_event, "event");

    // Frame
    private_nh.param<std::string>("frame", configuration_.frame, "");

    // Ethernet connection
    private_nh.param<std::string>("ethernet_peerAddress", configuration_.ethernet_peerAddress, "127.0.0.1");
    private_nh.param<int>("ethernet_peerPort", configuration_.ethernet_peerPort, 55555);
    private_nh.param<int>("ethernet_bufferSize", configuration_.ethernet_bufferSize, 0);
    private_nh.param<int>("ethernet_reconnectInterval", configuration_.ethernet_reconnectInterval, 500);

    /// Subscribing & Publishing
    subscriber_ethernet_ = nh.subscribe(configuration_.topic_hostToBus, 100, &Node::rosCallback_ethernet, this);
    publisher_ethernet_packet_ = nh.advertise<ethernet_msgs::Packet>(configuration_.topic_busToHost, 100);
    publisher_ethernet_event_ = nh.advertise<ethernet_msgs::Event>(configuration_.topic_event, 100, true);

    /// Bring up socket
    // Initialiazation
    socket_ = new QTcpSocket(this);
    connect(socket_, SIGNAL(readyRead()), this, SLOT(slotEthernetNewData()));
    connect(socket_, SIGNAL(connected()), this, SLOT(slotEthernetConnected()));
    connect(socket_, SIGNAL(disconnected()), this, SLOT(slotEthernetDisconnected()));
#if (QT_VERSION >= QT_VERSION_CHECK(5, 15, 0))
    connect(socket_, SIGNAL(errorOccured(QAbstractSocket::SocketError)), this, SLOT(slotEthernetError(QAbstractSocket::SocketError)));
#endif

    // Publish unconnected state
    slotEthernetDisconnected();

    // Configuration
    if (configuration_.ethernet_bufferSize > 0)
        socket_->setReadBufferSize(configuration_.ethernet_bufferSize);

    // Initial connection
    ROS_INFO("Connecting to %s:%u ...", configuration_.ethernet_peerAddress.data(), configuration_.ethernet_peerPort);
    socket_->connectToHost(QString::fromStdString(configuration_.ethernet_peerAddress), configuration_.ethernet_peerPort);

    /// Initialize timer for reconnecting on connection loss
    connect(&timer_, SIGNAL(timeout()), this, SLOT(slotTimer()));
    if (configuration_.ethernet_reconnectInterval > 0)
    {
        timer_.setInterval(configuration_.ethernet_reconnectInterval);
        timer_.start();
    }
}

Node::~Node()
{
    delete socket_;
}

void Node::rosCallback_ethernet(const ethernet_msgs::Packet::ConstPtr &msg)
{
    socket_->write(reinterpret_cast<const char*>(msg->payload.data()), msg->payload.size());
}

void Node::slotEthernetNewData()
{
    while (socket_->bytesAvailable())
    {
        packet.header.stamp = ros::Time::now();
        packet.header.frame_id = configuration_.frame;

        packet.sender_ip = ethernet_msgs::arrayByNativeIp4(socket_->peerAddress().toIPv4Address());
        packet.sender_port = socket_->peerPort();
        packet.receiver_ip = ethernet_msgs::arrayByNativeIp4(socket_->localAddress().toIPv4Address());
        packet.receiver_port = socket_->localPort();

        packet.payload.clear();
        packet.payload.reserve(socket_->bytesAvailable());
        QByteArray payload = socket_->readAll();
        std::copy(payload.constBegin(), payload.constEnd(), std::back_inserter(packet.payload));

        publisher_ethernet_packet_.publish(packet);
    }
}

void Node::slotEthernetConnected()
{
    ethernet_msgs::Event event;

    event.header.stamp = ros::Time::now();
    event.header.frame_id = configuration_.frame;

    event.type = ethernet_msgs::EventType::CONNECTED;

    publisher_ethernet_event_.publish(event);

    ROS_INFO("Connected.");
}

void Node::slotEthernetDisconnected()
{
    ethernet_msgs::Event event;

    event.header.stamp = ros::Time::now();
    event.header.frame_id = configuration_.frame;

    event.type = ethernet_msgs::EventType::DISCONNECTED;

    publisher_ethernet_event_.publish(event);

    ROS_INFO("Disconnected.");
}

void Node::slotEthernetError(int error_code)
{
    ethernet_msgs::Event event;

    event.header.stamp = ros::Time::now();
    event.header.frame_id = configuration_.frame;

    event.type = ethernet_msgs::EventType::SOCKETERROR;
    event.value = error_code;

    publisher_ethernet_event_.publish(event);

    ROS_WARN("Connection error occured, socket error code: %i", error_code);
}

void Node::slotTimer()
{
    if (socket_->state() != QAbstractSocket::ConnectedState)    // also covers deadlocks in other states like "ConnectingState"
        socket_->connectToHost(QString::fromStdString(configuration_.ethernet_peerAddress), configuration_.ethernet_peerPort);
}
