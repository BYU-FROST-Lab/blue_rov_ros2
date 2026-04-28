import socket, struct

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(('', 4747))  # adjust port if different

# join multicast if needed
import struct
mreq = struct.pack("4sL", socket.inet_aton("224.0.0.96"), socket.INADDR_ANY)
sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

data, addr = sock.recvfrom(65535)
print(f"Magic: {data[:4]}")
print(f"First 32 bytes: {data[:32].hex()}")
print(f"Total length: {len(data)}")