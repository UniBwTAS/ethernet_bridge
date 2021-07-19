# ethernet_msgs
## _Ethernet Packet Definitions for ROS_

![Status Badge](https://github.com/UniBwTAS/ethernet_msgs/actions/workflows/basic-build-ci.yml/badge.svg?branch=master)

ethernet_msgs are ROS message definitions for Ethernet packets. They can be used to insert network abstraction layers in ROS processing chains and to separate Ethernet interfaces from parser nodes. As Ethernet data is usually compressed, it is conceivable to store it in ROS bags instead of processed or parsed device data. By this means, the size of ROS bags is kept minimal while providing a longterm-stable, generic data interface. See `ethernet_bridge` for a set of ROS interface nodes and more details on the usage.

## Features

- Each packet contains the payload and the sender and receiver information of an network packet
- Standard packet definition for IPv4-based data transfers with focus on low overhead and user-friendliness
- Extended, generic packet definition which can be used for IPv6
- Supporting both packet and stream based protocols like TCP, UDP
- Event message types for connection events (e.g. TCP connection established event)
- Longterm-stable: the definitions have been used and tested for various applications, and we avoid future modifications to ensure longterm bag compatibility

## Installation

No dependencies to care for, just compile and source.

## Usage and Examples

Please have a look on `ethernet_bridge`.