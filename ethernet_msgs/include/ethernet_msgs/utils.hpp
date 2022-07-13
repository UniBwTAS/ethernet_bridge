#pragma once

#include <ethernet_msgs/msg/packet.hpp>

namespace ethernet_msgs
{
static inline uint32_t nativeIp4ByArray(msg::Packet::_sender_ip_type const& array)
{
    return 256*256*256 * static_cast<uint8_t>(array.at(0)) + 256*256 * static_cast<uint8_t>(array.at(1)) + 256 * static_cast<uint8_t>(array.at(2)) + static_cast<uint8_t>(array.at(3));
}

static inline msg::Packet::_sender_ip_type arrayByNativeIp4(uint32_t const& number)
{
    return msg::Packet::_sender_ip_type{static_cast<uint8_t>(number >> 24), static_cast<uint8_t>(number >> 16), static_cast<uint8_t>(number >> 8), static_cast<uint8_t>(number)};
}

}
