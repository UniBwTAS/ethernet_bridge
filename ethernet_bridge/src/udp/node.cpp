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
    private_nh.param<int>("ethernet_receiveBufferSize", configuration_.ethernet_receiveBufferSize, 20*1024*1024);

    /// Subscribing & Publishing
    ros::TransportHints t = ros::TransportHints().tcp().tcpNoDelay(true);
    subscriber_ethernet_ = nh.subscribe(configuration_.topic_hostToBus, 100, &Node::rosCallback_ethernet, this, t);
    publisher_ethernet_packet_ = nh.advertise<ethernet_msgs::Packet>(configuration_.topic_busToHost, 100);
    publisher_ethernet_event_ = nh.advertise<ethernet_msgs::Event>(configuration_.topic_event, 100, true);

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

    // Set receive buffer size
    if (configuration_.ethernet_receiveBufferSize >= 0)
        socket_->setSocketOption(QAbstractSocket::ReceiveBufferSizeSocketOption, configuration_.ethernet_receiveBufferSize);

    /// Console feedback
    ROS_INFO("Binding to %s:%u -> %s", QHostAddress(QString::fromStdString(configuration_.ethernet_bindAddress)).toString().toLatin1().data(), configuration_.ethernet_bindPort, success?"ok":"failed");
}

Node::~Node()
{
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
        //  ethernet_msgs::Packet packet; // moved to private member for optimization

        packet.header.stamp = ros::Time::now();
        packet.header.frame_id = configuration_.frame;

        packet.sender_ip = ethernet_msgs::arrayByNativeIp4(datagram.senderAddress().toIPv4Address());
        packet.sender_port = datagram.senderPort();
        packet.receiver_ip = ethernet_msgs::arrayByNativeIp4(datagram.destinationAddress().toIPv4Address());
        packet.receiver_port = datagram.destinationPort();

        packet.payload = std::vector<uint8_t>(datagram.data().cbegin(), datagram.data().cend());

        publisher_ethernet_packet_.publish(packet);
    }
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
