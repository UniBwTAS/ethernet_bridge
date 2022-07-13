#include <QCoreApplication>
#include <rclcpp/rclcpp.hpp>
#include <libros2qt/qt_executor.h>

#include "node.h"

int main(int argc, char* argv[]) {

    QCoreApplication a(argc, argv);
    rclcpp::init(argc, argv);

    auto server = std::make_shared<Node>("ethernet_bridge_udp");

    QtExecutor executor;
    executor.add_node(server);

    executor.start();

    auto res = a.exec();
    rclcpp::shutdown();
    return res;
}