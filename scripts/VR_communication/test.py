import socket

# === Parametri del server ===
SERVER_IP = "10.101.120.101"  # <--- Inserisci l'IP del server
SERVER_PORT = 5005

# === Connessione TCP ===
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

try:
    print(f"Connecting to {SERVER_IP}:{SERVER_PORT}...")
    sock.connect((SERVER_IP, SERVER_PORT))
    print("Connected. Waiting for msgs...\n")

    buffer = b""
    while True:
        data = sock.recv(1024)
        if not data:
            print("Connection closed by the server.")
            break

        buffer += data
        while b"\n" in buffer:
            line, buffer = buffer.split(b"\n", 1)
            print("Message retrieved:", line.decode().strip())

except Exception as e:
    print(f"Error: {e}")
finally:
    sock.close()
    print("Socket closed.")
