## A single IPv4 network packet.

# Header
Header header

# Connection Information
uint8[4] sender_ip
uint16   sender_port
uint8[4] receiver_ip
uint16   receiver_port

# Data Payload
uint8[] payload
