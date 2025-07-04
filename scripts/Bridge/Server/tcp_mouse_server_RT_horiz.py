import socket
import tkinter as tk
import threading
import time

# === TCP parameters ===
HOST = '0.0.0.0'
PORT = 5005

# === Timing parameters ===
TARGET_FPS = 60
FRAME_TIME = 1.0 / TARGET_FPS  # ~0.0167 seconds

# === A4 Landscape window (wider than taller) ===
# A4 aspect ratio is approximately 1.414:1 (297mm x 210mm)
WIN_WIDTH = 1000  # Wider
WIN_HEIGHT = 707  # Taller (1000/1.414 ≈ 707)

# === Coordinate system parameters ===
# New coordinate system: (1400,1000) top-left, (0,0) bottom-right
COORD_MAX_X = 1400
COORD_MAX_Y = 1000

# === Stato ===
client_conn = None
last_x = None
last_y = None
last_mapped_x = None
last_mapped_y = None
sending = False
coordinate_sender_active = False


def coordinate_sender_thread():
    """
    Continuous thread that sends coordinates at 60Hz when drawing is active.
    """
    global coordinate_sender_active, last_mapped_x, last_mapped_y
    
    print("[COORDINATE SENDER] Thread started - 60Hz transmission")
    
    while coordinate_sender_active:
        start_time = time.time()
        
        if sending and last_mapped_x is not None and last_mapped_y is not None:
            send_custom_coordinates(last_mapped_x, last_mapped_y)
        
        # Calculate sleep time to maintain 60Hz
        elapsed = time.time() - start_time
        sleep_time = max(0, FRAME_TIME - elapsed)
        time.sleep(sleep_time)


def send_custom_coordinates(x, y):
    """
    Send coordinates in the format expected by the receiver: "x,y\n" (no parentheses)
    Coordinates range: X from 0-1400, Y from 0-1000
    """
    if client_conn:
        try:
            # Format: "x,y\n" to match receiver expectations
            msg = f"{x:.1f},{y:.1f}\n"
            client_conn.sendall(msg.encode())
        except Exception as e:
            print(f"[ERRORE] Invio fallito: {e}")


def map_to_custom_coords(px, py):
    """
    Map canvas coordinates to the new coordinate system:
    - Top Left = (1400, 1000)
    - Top Right = (0, 1000) 
    - Bottom Left = (1400, 0)
    - Bottom Right = (0, 0)
    
    GUI origin: (0,0) at top-left
    Custom origin: (0,0) at bottom-right
    """
    # Map X: left edge (px=0) → 1400, right edge (px=WIN_WIDTH) → 0
    mapped_x = (WIN_WIDTH - px) * COORD_MAX_X / WIN_WIDTH
    
    # Map Y: top edge (py=0) → 1000, bottom edge (py=WIN_HEIGHT) → 0  
    mapped_y = (WIN_HEIGHT - py) * COORD_MAX_Y / WIN_HEIGHT
    
    return mapped_x, mapped_y


def on_mouse_press(event):
    global sending, last_x, last_y, last_mapped_x, last_mapped_y
    sending = True
    last_x, last_y = event.x, event.y
    # Calculate initial mapped coordinates
    last_mapped_x, last_mapped_y = map_to_custom_coords(event.x, event.y)


def on_mouse_release(event):
    global sending
    sending = False


def on_mouse_drag(event):
    global last_x, last_y, last_mapped_x, last_mapped_y
    if sending:
        # Draw line on canvas for visual feedback
        canvas.create_line(last_x, last_y, event.x, event.y, fill="blue", width=2)
        
        # Convert and store coordinates (the sender thread will transmit them at 60Hz)
        last_mapped_x, last_mapped_y = map_to_custom_coords(event.x, event.y)
        
        # Update position for debug info
        canvas.delete("debug_info")
        debug_text = f"Pixel: ({event.x}, {event.y}) | Custom: ({last_mapped_x:.1f}, {last_mapped_y:.1f})"
        canvas.create_text(10, 10, anchor="nw", text=debug_text, tags="debug_info", fill="red")
        
        # Add corner reference for debugging
        canvas.delete("corner_info")
        corner_text = "Custom Map: TL(1400,1000) TR(0,1000) BL(1400,0) BR(0,0)"
        canvas.create_text(10, 30, anchor="nw", text=corner_text, tags="corner_info", fill="green")
        
        # Add format info for debugging
        canvas.delete("format_info")
        format_text = f"Sending @ 60Hz: {last_mapped_x:.1f},{last_mapped_y:.1f} | A4 Landscape"
        canvas.create_text(10, 50, anchor="nw", text=format_text, tags="format_info", fill="purple")
        
        last_x, last_y = event.x, event.y


def tcp_server():
    global client_conn, coordinate_sender_active
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Allow port reuse
    server.bind((HOST, PORT))
    server.listen(1)
    print(f"[TCP SERVER] In ascolto su {HOST}:{PORT}...")
    client_conn, addr = server.accept()
    print(f"[TCP SERVER] Connessione da {addr}")
    print("[TCP SERVER] Ora inviando coordinate nel formato: x,y (senza parentesi)")
    print("[TCP SERVER] Sistema coordinate: (0,0)=bottom-right, (1400,1000)=top-left")
    print("[TCP SERVER] Frequenza di invio: 60Hz")
    
    # Start coordinate sender thread
    coordinate_sender_active = True
    sender_thread = threading.Thread(target=coordinate_sender_thread, daemon=True)
    sender_thread.start()


def clear_canvas():
    """Clear the drawing canvas"""
    canvas.delete("all")
    # Redraw instructions
    canvas.create_text(WIN_WIDTH//2, WIN_HEIGHT//2, 
                      text="Move the mouse to draw - A4 Landscape Format\nCoordinates: (0,0) bottom-right, (1400,1000) top-left\nTransmission: 60Hz", 
                      justify=tk.CENTER, fill="gray")


def start_gui():
    global canvas
    root = tk.Tk()
    root.title("A4 Coordinate Server @ 60Hz - Format: x,y (0-1400, 0-1000)")
    root.geometry(f"{WIN_WIDTH}x{WIN_HEIGHT+50}")

    # Create main frame
    main_frame = tk.Frame(root)
    main_frame.pack(fill=tk.BOTH, expand=True)

    # Create canvas with A4 landscape proportions
    canvas = tk.Canvas(main_frame, bg="white", width=WIN_WIDTH, height=WIN_HEIGHT)
    canvas.pack()
    
    # Bind mouse events
    canvas.bind('<ButtonPress-1>', on_mouse_press)
    canvas.bind('<ButtonRelease-1>', on_mouse_release)
    canvas.bind('<B1-Motion>', on_mouse_drag)

    # Add initial instructions
    canvas.create_text(WIN_WIDTH//2, WIN_HEIGHT//2, 
                      text="Move the mouse to draw - A4 Landscape Format\nCoordinates: (0,0) bottom-right, (1400,1000) top-left\nTransmission: 60Hz", 
                      justify=tk.CENTER, fill="gray")

    # Create button frame
    button_frame = tk.Frame(root)
    button_frame.pack(fill=tk.X, pady=5)
    
    # Add clear button
    clear_button = tk.Button(button_frame, text="Pulisci Canvas", command=clear_canvas)
    clear_button.pack(side=tk.LEFT, padx=5)
    
    # Add info label
    info_label = tk.Label(button_frame, 
                         text="A4 Landscape @ 60Hz: (0,0) bottom-right, (1400,1000) top-left | Format: x,y", 
                         fg="blue")
    info_label.pack(side=tk.RIGHT, padx=5)

    # Cleanup on window close
    def on_closing():
        global coordinate_sender_active
        coordinate_sender_active = False
        root.destroy()
    
    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()


if __name__ == "__main__":
    print("=== TCP Mouse Pointer Server (A4 Landscape @ 60Hz) ===")
    print("Window format: A4 Landscape (1000x707)")
    print("Coordinate system: X(0-1400), Y(0-1000)")
    print("(0,0) = bottom-right, (1400,1000) = top-left")
    print("Message format: x,y (no parentheses)")
    print("Transmission frequency: 60Hz continuous")
    print("=====================================================")
    
    threading.Thread(target=tcp_server, daemon=True).start()
    start_gui()