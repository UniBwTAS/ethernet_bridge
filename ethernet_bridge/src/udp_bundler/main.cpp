#include <ros/ros.h>
#include <QCoreApplication>
#include <librosqt/QRosCallBackQueue.h>

#include "node.h"

int main(int argc, char **argv)
{
    /// Initialization of ROS and Qt
    QCoreApplication app(argc, argv);
    QRosCallBackQueue::replaceGlobalQueue();
    ros::init(argc, argv, "ethernet_bridge_udp_bundler", ros::init_options::NoSigintHandler);

    /// Initialization of the node
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    /// Start node
    Node node(nh, private_nh);

    /// Run node
    app.exec();

    return 0;
}
