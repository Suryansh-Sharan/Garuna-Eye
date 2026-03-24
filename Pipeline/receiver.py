import socket
import json

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", 6000))

print("📍 Waiting for GPS data...")

while True:
    data, addr = sock.recvfrom(1024)
    print(json.loads(data.decode()))
