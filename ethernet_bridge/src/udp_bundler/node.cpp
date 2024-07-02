#include "node.h"
#include <QUdpSocket>
#include <QNetworkDatagram>
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
    private_nh.param<std::string>("ethernet_bindAddress", configuration_.ethernet_bindAddress, "0.0.0.0");
    private_nh.param<int>("ethernet_bindPort", configuration_.ethernet_bindPort, 55555);

    // Bundler send triggers
    private_nh.param<int>("trigger_numberOfPackets", configuration_.trigger_numberOfPackets, 50);
    private_nh.param<int>("trigger_maximumPacketAge", configuration_.trigger_maximumPacketAge, 10);
    private_nh.param<int>("trigger_maximumIdleTime", configuration_.trigger_maximumIdleTime, 2);

    /// Subscribing & Publishing
    subscriber_ethernet_ = nh.subscribe(configuration_.topic_hostToBus, 100, &Node::rosCallback_ethernet, this);
    publisher_ethernet_packets_ = nh.advertise<ethernet_msgs::Packets>(configuration_.topic_busToHost, 100);
    publisher_ethernet_event_ = nh.advertise<ethernet_msgs::Event>(configuration_.topic_event, 100, true);

    /// Initialize Bundler
    if (configuration_.trigger_numberOfPackets > 0)
        bundler_buffer_.packets.reserve(configuration_.trigger_numberOfPackets);

    connect(&bundler_age_timer_, SIGNAL(timeout()), this, SLOT(slotBundlerAgeExceeded()));
    if (configuration_.trigger_maximumPacketAge > 0)
    {
        bundler_age_timer_.setInterval(configuration_.trigger_maximumPacketAge);
        bundler_age_timer_.setSingleShot(true);
    }

    connect(&bundler_idle_timer_, SIGNAL(timeout()), this, SLOT(slotBundlerIdleExceeded()));
    if (configuration_.trigger_maximumIdleTime > 0)
    {
        bundler_idle_timer_.setInterval(configuration_.trigger_maximumIdleTime);
        bundler_idle_timer_.setSingleShot(true);
    }

    /// Initialize socket
    socket_ = new QUdpSocket(this);
    connect(socket_, SIGNAL(readyRead()), this, SLOT(slotEthernetNewData()));
    connect(socket_, SIGNAL(connected()), this, SLOT(slotEthernetConnected()));
    connect(socket_, SIGNAL(disconnected()), this, SLOT(slotEthernetDisconnected()));
#if (QT_VERSION >= QT_VERSION_CHECK(5, 15, 0))
    connect(socket_, SIGNAL(errorOccured(QAbstractSocket::SocketError)), this, SLOT(slotEthernetError(QAbstractSocket::SocketError)));
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

    ROS_INFO("Binding to %s:%u -> %s", QHostAddress(QString::fromStdString(configuration_.ethernet_bindAddress)).toString().toLatin1().data(), configuration_.ethernet_bindPort, success?"ok":"failed");
}

Node::~Node()
{
    if (bundler_buffer_.packets.size() > 0)
        transmitBuffer();
    delete socket_;
}

void Node::rosCallback_ethernet(const ethernet_msgs::Packet::ConstPtr &msg)
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

        // ethernet_msgs::Packet& packet = bundler_buffer_.packets.emplace_back(); only supported >= C++17, so instead we do:
        bundler_buffer_.packets.emplace_back();
        ethernet_msgs::Packet& packet = bundler_buffer_.packets.back();

        packet.header.stamp = ros::Time::now();
        packet.header.frame_id = configuration_.frame;

        packet.sender_ip = ethernet_msgs::arrayByNativeIp4(datagram.senderAddress().toIPv4Address());
        packet.sender_port = datagram.senderPort();
        packet.receiver_ip = ethernet_msgs::arrayByNativeIp4(datagram.destinationAddress().toIPv4Address());
        packet.receiver_port = datagram.destinationPort();

        packet.payload = std::vector<uint8_t>(datagram.data().cbegin(), datagram.data().cend());

        // Is it the first packet in the buffer? Then start the maximum age timer.
        if ((configuration_.trigger_maximumPacketAge > 0) && (bundler_buffer_.packets.size() == 1))
            bundler_age_timer_.start();
    }

    // We have received a set of UDP packets. Is the desired number of packets per ROS message reached or even exceeded?
    if ((configuration_.trigger_numberOfPackets > 0) && (bundler_buffer_.packets.size() >= configuration_.trigger_numberOfPackets))
        transmitBuffer();
    else
    {
        // Restart (= start) the maximum idle timer.
        if (configuration_.trigger_maximumIdleTime > 0)
            bundler_idle_timer_.start();
    }
}

void Node::slotBundlerAgeExceeded()
{
    transmitBuffer();
}

void Node::slotBundlerIdleExceeded()
{
    transmitBuffer();
}

void Node::transmitBuffer()
{
    if (bundler_buffer_.packets.size() == 0)
    {
        ROS_ERROR("sending an empty buffer...should not happen. program faulty.");
        // QTimers should not be in race conflict as SIGNALs are processed sequentially by Qt: first, timer is stopped, then it won't be checked to emit signals.
    }

    publisher_ethernet_packets_.publish(bundler_buffer_);
    bundler_buffer_.packets.clear();

    bundler_age_timer_.stop();
    bundler_idle_timer_.stop();
}

// Note: UDP sockets are usually not "connected" or "disconnected"
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
