# ethernet_bridge
## _Ethernet Bridge for ROS_

ethernet_bridges are a set of ROS nodes to bridge network interfaces from and to the ROS message definitions [`ethernet_msgs`](ethernet_msgs/). The Ethernet bridge can be used to insert network abstraction layers in ROS processing chains and to separate Ethernet interfaces from parser nodes. As Ethernet data is usually compressed, it is conceivable to store raw Ethernet data in ROS bags instead of processed or parsed device data. By this means, the size of ROS bags is kept minimal while providing a longterm-stable, generic data interface.

## Advantages of an Ethernet data layer

- Ethernet data is usually compressed raw data and perfectly suited to be stored in ROS bags
- Generic and longterm-stable: ROS bags with Ethernet data can be processed with any kind of (future) parser
- A ROS bag with Ethernet raw data is comparable to a pcap file; the Ethernet packets can be re-sent over the network to feed applications without ROS interface
- High reusability: parsers can be bugfixed and extended without the need of re-recording data (in contrast to storing processed data)
- Multiple concurrent nodes can process a data stream simultaneously

## Features

- Low additional ROS overhead (the standard UDP node is suitable to abstract 128-diode laser scanners with ~6000 UDP packets per second, ~8 MBytes/s with < 5% CPU overhead)
- All bridges support bidrectional online transfers
- Each packet contains the ports and IPs of sender and receiver
- Event messaging (e.g. TCP connection established signal)
- Tested for various applications and devices (Lidar scanners, Radar sensors, INS)
- We avoid any further modifications on our existing `ethernet_msgs` to ensure longterm bag capability
- No additional system dependencies, uses only standard ROS desktop dependencies (Qt)

## Currently available bridges
- UDP node

## ROS Package Dependencies

- `ethernet_msgs`
- `librosqt`

## Nodes Description
### _udp_: Standard UDP Bridge
This is the standard bridge for UDP communication and focuses on most application use cases except of transfers with a very high packet load.

#### Options

- `topic_*`: ROS topic specific configuration segment
  - `topic_busToHost`: ROS topic which receives Ethernet packets (published by bridge)
  - `topic_hostToBus`: ROS topic which sends Ethernet packets (subscribed by bridge)
  - `topic_event`: ROS topic providing communication events (published by bridge)
- `ethernet_*`: Ethernet interface specific configuration segment
  - `ethernet_bindAddress`: specific local IP bind address (optional)
  - `ethernet_bindPort`: local listening port

#### Application Example

This is an extract of a launch file for a BroadR-Reach-based radar sensor network with multiple devices for both bidirectional point-to-point and broadcast communication:
```
  <!-- Node parameters -->
  <arg name="topic_ethernet"        default="/bus/ethernet/umrr/udp55555" />
  <arg name="ethernet_ip"           default="0.0.0.0" />
  <arg name="ethernet_port"         default="55555" />
  
  <!-- Ethernet bridge node -->
  <node pkg="ethernet_bridge" type="udp" name="udp">
    <param name="topic_busToHost"       value="$(arg topic_ethernet)/bus_to_host" />
    <param name="topic_hostToBus"       value="$(arg topic_ethernet)/host_to_bus" />
    <param name="topic_event"           value="$(arg topic_ethernet)/event" />
    <param name="ethernet_bindAddress"  value="$(arg ethernet_ip)" />
    <param name="ethernet_bindPort"     value="$(arg ethernet_port)" />
  </node>
```

## Contributions

We look forward to contributions to `ethernet_bridge` and `ethernet_msgs`. Please note that modifications on existing `ethernet_msgs` definitions would impair longterm stability and are avoided.
