import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(("192.168.1.103", 5005))  # 127.0.0.1   192.168.1.111

while True:
    data = sock.recv(1024)
    if not data:
        break
    print(data.decode(), end="")  # Stampa esattamente quello che arriva
