import socket
import threading
import sys
import tty
import termios

# Server configuration
SERVER_IP = "192.168.1.100"
SERVER_PORT = 5005

# Global socket for receiving
receive_sock = None

def getch():
    """Get a single character from stdin without pressing Enter"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def send_command(command):
    """Send a command to the server using a separate connection"""
    try:
        send_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        send_sock.connect((SERVER_IP, SERVER_PORT))
        send_sock.send(command.encode() + b"\n")
        send_sock.close()
        print(f"✅ Sent: {command}")
    except Exception as e:
        print(f"❌ Failed to send {command}: {e}")

def keyboard_listener():
    """Listen for keyboard input and send commands"""
    print("--- Keyboard Controls ---")
    print("G: Send GO command")
    print("S: Send STOP command") 
    print("Q: Quit")
    print("------------------------")
    
    while True:
        try:
            key = getch().lower()
            
            if key == 'g':
                send_command("GO")
            elif key == 's':
                send_command("STOP")
            elif key == 'q':
                print("Quitting...")
                break
            else:
                print(f"Unknown key: {key}")
                
        except KeyboardInterrupt:
            print("\nQuitting...")
            break
        except Exception as e:
            print(f"Keyboard error: {e}")
            break

def tcp_receiver():
    """Receive data from the server continuously"""
    global receive_sock
    
    try:
        receive_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        receive_sock.connect((SERVER_IP, SERVER_PORT))
        print(f"✅ Connected to {SERVER_IP}:{SERVER_PORT} for receiving")
        
        while True:
            try:
                data = receive_sock.recv(1024)
                if not data:
                    print("❌ Connection closed by server")
                    break
                print(data.decode(), end="")  # Print exactly what arrives
                
            except Exception as e:
                print(f"❌ Receive error: {e}")
                break
                
    except Exception as e:
        print(f"❌ Connection failed: {e}")
    finally:
        if receive_sock:
            receive_sock.close()
            print("Receive connection closed")

def main():
    print("TCP Test Client with GO/STOP Commands")
    print("====================================")
    
    # Start receiving thread
    receive_thread = threading.Thread(target=tcp_receiver, daemon=True)
    receive_thread.start()
    
    # Start keyboard listener (blocking)
    try:
        keyboard_listener()
    except KeyboardInterrupt:
        print("\nShutting down...")
    
    # Close connections
    if receive_sock:
        receive_sock.close()
    
    print("Client terminated")

if __name__ == "__main__":
    main()