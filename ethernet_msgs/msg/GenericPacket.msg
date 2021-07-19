## A single generic (IPv4, IPv6, ...) network packet.

# Header
Header header

# Protocol Type
uint8 protocol          # enum defined in ProtocolType.msg

# Connection Information
uint8[] sender_ip       # size: IPv4 = 4, IPv6 = 16
uint16  sender_port
uint8[] receiver_ip     # size: IPv4 = 4, IPv6 = 16
uint16  receiver_port

# Meta Information
uint8[] meta            # arbitrary meta information

# Data Payload
uint8[] payload
