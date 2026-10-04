import socket

WSL_IP = "172.21.54.182"
PORT = 12345

receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
receiver.bind(("0.0.0.0", PORT))

sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
print(f"Relaying UDP port {PORT} to WSL {WSL_IP}:{PORT}")

while True:
    packet, source = receiver.recvfrom(2048)
    sender.sendto(packet, (WSL_IP, PORT))