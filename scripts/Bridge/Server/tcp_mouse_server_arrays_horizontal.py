import socket
import tkinter as tk
import threading
import time

# === TCP parameters ===
HOST = '0.0.0.0'
PORT = 5005

# === Virtual window ===
WIN_WIDTH = 800
WIN_HEIGHT = 600

# === NEW COORDINATE SYSTEM PARAMETERS ===
# Updated to match the new (1400, 1000) coordinate system
VR_WIDTH = 1400   # Virtual canvas width
VR_HEIGHT = 1000  # Virtual canvas height

# === State ===
client_conn = None
current_stroke = []  # Current stroke being drawn
all_strokes = []     # All completed strokes
drawing = False      # Whether currently drawing a stroke


def send_all_strokes():
    """
    Send all collected strokes in the format: x1,y1 x2,y2 x3,y3 x4,y4 ...
    Each stroke is sent as a separate message ending with \n
    """
    if not client_conn:
        print("[ERROR] No client connected!")
        return
    
    if not all_strokes:
        print("[INFO] No strokes to send!")
        return
    
    try:
        stroke_count = 0
        for stroke in all_strokes:
            if len(stroke) >= 2:  # Need at least 2 points for a stroke
                # Format: x1,y1 x2,y2 x3,y3 ...
                coord_pairs = []
                for x, y in stroke:
                    coord_pairs.append(f"{x:.1f},{y:.1f}")
                
                message = " ".join(coord_pairs) + "\n"
                client_conn.sendall(message.encode())
                stroke_count += 1
                print(f"[SENT] Stroke {stroke_count}: {len(stroke)} points")
        
        print(f"[COMPLETED] Sent {stroke_count} strokes total")
        update_status_display()
        
    except Exception as e:
        print(f"[ERROR] Failed to send strokes: {e}")


def map_to_vr_coords(px, py):
    """
    Map canvas coordinates to the NEW VR coordinate system:
    - Top-left = (1400, 1000)     [GUI: (0, 0)]
    - Top-right = (0, 1000)       [GUI: (WIN_WIDTH, 0)]
    - Bottom-left = (1400, 0)     [GUI: (0, WIN_HEIGHT)]
    - Bottom-right = (0, 0)       [GUI: (WIN_WIDTH, WIN_HEIGHT)]
    
    GUI origin: (0,0) at top-left
    VR origin: (1400, 1000) at top-left
    """
    # Map X: GUI left (0) → VR left (1400), GUI right (WIN_WIDTH) → VR right (0)
    mapped_x = VR_WIDTH * (1.0 - px / WIN_WIDTH)
    
    # Map Y: GUI top (0) → VR top (1000), GUI bottom (WIN_HEIGHT) → VR bottom (0)  
    mapped_y = VR_HEIGHT * (1.0 - py / WIN_HEIGHT)
    
    return mapped_x, mapped_y


def on_mouse_press(event):
    global drawing, current_stroke
    drawing = True
    current_stroke = []  # Start new stroke
    # Add first point
    mapped_x, mapped_y = map_to_vr_coords(event.x, event.y)
    current_stroke.append((mapped_x, mapped_y))
    update_debug_display(event.x, event.y, mapped_x, mapped_y)


def on_mouse_release(event):
    global drawing, current_stroke, all_strokes
    if drawing and len(current_stroke) > 0:
        # Add final point if different from last
        mapped_x, mapped_y = map_to_vr_coords(event.x, event.y)
        if len(current_stroke) == 0 or (mapped_x, mapped_y) != current_stroke[-1]:
            current_stroke.append((mapped_x, mapped_y))
        
        # Save completed stroke
        all_strokes.append(current_stroke.copy())
        print(f"[STROKE COMPLETED] Points: {len(current_stroke)}")
        
    drawing = False
    current_stroke = []
    update_status_display()


def on_mouse_drag(event):
    global current_stroke
    if drawing:
        # Draw line on canvas for visual feedback
        if len(current_stroke) > 0:
            # Get last point in pixel coordinates for drawing
            last_mapped_x, last_mapped_y = current_stroke[-1]
            # Convert back to pixel coordinates for drawing
            last_px = WIN_WIDTH * (1.0 - last_mapped_x / VR_WIDTH)
            last_py = WIN_HEIGHT * (1.0 - last_mapped_y / VR_HEIGHT)
            canvas.create_line(last_px, last_py, event.x, event.y, fill="blue", width=2)
        
        # Add point to current stroke
        mapped_x, mapped_y = map_to_vr_coords(event.x, event.y)
        current_stroke.append((mapped_x, mapped_y))
        
        update_debug_display(event.x, event.y, mapped_x, mapped_y)


def update_debug_display(px, py, mapped_x, mapped_y):
    """Update debug information on canvas"""
    canvas.delete("debug_info")
    debug_text = f"GUI: ({px}, {py}) | VR: ({mapped_x:.1f}, {mapped_y:.1f})"
    canvas.create_text(10, 10, anchor="nw", text=debug_text, tags="debug_info", fill="red")
    
    canvas.delete("corner_info")
    corner_text = f"VR Map: TL({VR_WIDTH},{VR_HEIGHT}) TR(0,{VR_HEIGHT}) BL({VR_WIDTH},0) BR(0,0)"
    canvas.create_text(10, 30, anchor="nw", text=corner_text, tags="corner_info", fill="green")


def update_status_display():
    """Update stroke count and status information"""
    canvas.delete("status_info")
    total_points = sum(len(stroke) for stroke in all_strokes)
    current_points = len(current_stroke) if current_stroke else 0
    
    status_text = f"Strokes: {len(all_strokes)} | Total Points: {total_points}"
    if drawing:
        status_text += f" | Current: {current_points} points"
    
    canvas.create_text(10, 50, anchor="nw", text=status_text, tags="status_info", fill="purple")
    
    canvas.delete("format_info")
    format_text = "Format: x1,y1 x2,y2 x3,y3 ... (one stroke per message)"
    canvas.create_text(10, 70, anchor="nw", text=format_text, tags="format_info", fill="orange")


def tcp_server():
    global client_conn
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    server.listen(1)
    print(f"[TCP SERVER] Listening on {HOST}:{PORT}...")
    client_conn, addr = server.accept()
    print(f"[TCP SERVER] Connection from {addr}")
    print(f"[TCP SERVER] Ready to send stroke arrays in format: x1,y1 x2,y2 x3,y3 ...")
    print(f"[TCP SERVER] VR coordinate system: {VR_WIDTH}x{VR_HEIGHT}")
    # Update GUI to show connection status
    canvas.after(100, lambda: canvas.create_text(WIN_WIDTH-10, 10, anchor="ne", 
                                                text="CLIENT CONNECTED", 
                                                tags="connection_status", fill="green"))


def clear_canvas():
    """Clear the drawing canvas and reset stroke data"""
    global all_strokes, current_stroke
    canvas.delete("all")
    all_strokes = []
    current_stroke = []
    print("[CLEARED] All strokes cleared")
    
    # Redraw instructions
    canvas.create_text(WIN_WIDTH//2, WIN_HEIGHT//2, 
                      text="Draw with mouse, collect strokes\nClick 'Send Strokes' to transmit all", 
                      justify=tk.CENTER, fill="gray")
    update_status_display()


def clear_strokes_only():
    """Clear only the stroke data, keep the visual canvas"""
    global all_strokes, current_stroke
    all_strokes = []
    current_stroke = []
    print("[CLEARED] Stroke data cleared (canvas kept)")
    update_status_display()


def test_corner_coordinates():
    """Test function to print the corner coordinates"""
    print("\n=== TESTING CORNER COORDINATES ===")
    corners = [
        (0, 0, "Top-Left GUI"),
        (WIN_WIDTH, 0, "Top-Right GUI"), 
        (0, WIN_HEIGHT, "Bottom-Left GUI"),
        (WIN_WIDTH, WIN_HEIGHT, "Bottom-Right GUI")
    ]
    
    for px, py, name in corners:
        vr_x, vr_y = map_to_vr_coords(px, py)
        print(f"{name}: GUI({px}, {py}) → VR({vr_x:.1f}, {vr_y:.1f})")
    print("===================================\n")


def start_gui():
    global canvas
    root = tk.Tk()
    root.title(f"VR Coordinate Server - Stroke Collection Mode ({VR_WIDTH}x{VR_HEIGHT})")
    root.geometry(f"{WIN_WIDTH}x{WIN_HEIGHT+100}")

    # Create main frame
    main_frame = tk.Frame(root)
    main_frame.pack(fill=tk.BOTH, expand=True)

    # Create canvas
    canvas = tk.Canvas(main_frame, bg="white", width=WIN_WIDTH, height=WIN_HEIGHT)
    canvas.pack()
    
    # Bind mouse events
    canvas.bind('<ButtonPress-1>', on_mouse_press)
    canvas.bind('<ButtonRelease-1>', on_mouse_release)
    canvas.bind('<B1-Motion>', on_mouse_drag)

    # Add initial instructions
    canvas.create_text(WIN_WIDTH//2, WIN_HEIGHT//2, 
                      text=f"Draw with mouse, collect strokes\nVR System: {VR_WIDTH}x{VR_HEIGHT}\nClick 'Send Strokes' to transmit all", 
                      justify=tk.CENTER, fill="gray")

    # Create button frame
    button_frame = tk.Frame(root)
    button_frame.pack(fill=tk.X, pady=5)
    
    # Add buttons
    send_button = tk.Button(button_frame, text="Send Strokes", command=send_all_strokes, 
                           bg="lightgreen", font=("Arial", 10, "bold"))
    send_button.pack(side=tk.LEFT, padx=5)
    
    clear_button = tk.Button(button_frame, text="Clear Canvas", command=clear_canvas)
    clear_button.pack(side=tk.LEFT, padx=5)
    
    clear_data_button = tk.Button(button_frame, text="Clear Stroke Data", command=clear_strokes_only)
    clear_data_button.pack(side=tk.LEFT, padx=5)
    
    test_button = tk.Button(button_frame, text="Test Corners", command=test_corner_coordinates,
                           bg="lightyellow")
    test_button.pack(side=tk.LEFT, padx=5)
    
    # Add info label
    info_label = tk.Label(root, 
                         text=f"VR System: TL({VR_WIDTH},{VR_HEIGHT}) → BR(0,0) | Horizontal A4 Canvas Mode", 
                         fg="blue", font=("Arial", 9))
    info_label.pack(pady=2)

    # Initialize status display
    update_status_display()

    # Cleanup on window close
    def on_closing():
        if client_conn:
            try:
                client_conn.close()
            except:
                pass
        root.destroy()
    
    root.protocol("WM_DELETE_WINDOW", on_closing)
    
    # Print corner coordinate mapping on startup
    test_corner_coordinates()
    
    root.mainloop()


if __name__ == "__main__":
    print("=== TCP Mouse Pointer Server - VR Coordinate Simulator ===")
    print(f"VR coordinate system: {VR_WIDTH}x{VR_HEIGHT}")
    print(f"Top-left: ({VR_WIDTH}, {VR_HEIGHT})")
    print(f"Top-right: (0, {VR_HEIGHT})")
    print(f"Bottom-left: ({VR_WIDTH}, 0)")
    print(f"Bottom-right: (0, 0)")
    print("Message format: x1,y1 x2,y2 x3,y3 x4,y4 ... (one stroke per message)")
    print("Transmission: On-demand via 'Send Strokes' button")
    print("============================================================")
    
    threading.Thread(target=tcp_server, daemon=True).start()
    start_gui()