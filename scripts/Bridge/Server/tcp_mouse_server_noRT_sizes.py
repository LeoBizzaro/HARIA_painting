import socket
import tkinter as tk
from tkinter import ttk
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

# === Color definitions ===
BASE_COLORS = {
    "#16160F": 1,  # Black
    "#7A3D28": 2,  # Brown
    "#D82929": 3,  # Red
    "#E86E09": 4,  # Orange
    "#DBC416": 5,  # Yellow
    "#49C893": 6,  # Light Green
    "#02704D": 7,  # Dark Green
    "#0D2875": 8,  # Dark Blue
    "#0295D5": 9,  # Light Blue
    "#C0BDAE": 10, # White
}

# Color definitions with thickness variants only
COLORS = {
    # Thick colors ("b" at the end)
    "#16160Fb": 1, # Black
    "#7A3D28b": 2, # Brown
    "#D82929b": 3, # Red
    "#E86E09b": 4, # Orange
    "#DBC416b": 5, # Yellow
    "#49C893b": 6, # Light Green
    "#02704Db": 7, # Dark Green
    "#0D2875b": 8, # Dark Blue
    "#0295D5b": 9, # Light Blue
    "#C0BDAEb": 10, # White
    
    # Tight colors ("s" at the end)
    "#16160Fs": 11, # Black
    "#7A3D28s": 12, # Brown
    "#D82929s": 13, # Red
    "#E86E09s": 14, # Orange
    "#DBC416s": 15, # Yellow
    "#49C893s": 16, # Light Green
    "#02704Ds": 17, # Dark Green
    "#0D2875s": 18, # Dark Blue
    "#0295D5s": 19, # Light Blue
    "#C0BDAEs": 20, # White
}

# Thickness options
THICKNESS_OPTIONS = {
    "Thick": "b",
    "Tight": "s"
}

current_color_base = "#16160F"  # Base color without thickness suffix
current_thickness = "Thick"    # Current thickness selection (default to Thick)
stroke_colors = []  # Parallel list to all_strokes

# === State ===
client_conn = None
current_stroke = []  # Current stroke being drawn
all_strokes = []     # All completed strokes
drawing = False      # Whether currently drawing a stroke

def get_current_color_code():
    """Get the current color code with thickness suffix"""
    suffix = THICKNESS_OPTIONS[current_thickness]
    return current_color_base + suffix

def send_all_strokes():
    """
    Send all collected strokes in the format: x1,y1 x2,y2 x3,y3 ... #HEXCODE
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
        for stroke, color in zip(all_strokes, stroke_colors):
            if len(stroke) >= 2:
                coord_pairs = [f"{x:.1f},{y:.1f}" for x, y in stroke]
                message = f"{color} " + " ".join(coord_pairs) + "\n"
                client_conn.sendall(message.encode())
                stroke_count += 1
                print(f"[SENT] Stroke {stroke_count}: {len(stroke)} points + color {color}")
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
        
        # Save completed stroke with current color+thickness
        all_strokes.append(current_stroke.copy())
        stroke_colors.append(get_current_color_code())
        print(f"[STROKE COMPLETED] Points: {len(current_stroke)}, Color: {get_current_color_code()}")
        
    drawing = False
    current_stroke = []
    update_status_display()

def on_mouse_drag(event):
    global current_stroke
    if drawing:
        # Draw line on canvas for visual feedback with appropriate thickness
        line_width = get_visual_line_width()
        if len(current_stroke) > 0:
            # Get last point in pixel coordinates for drawing
            last_mapped_x, last_mapped_y = current_stroke[-1]
            # Convert back to pixel coordinates for drawing
            last_px = WIN_WIDTH * (1.0 - last_mapped_x / VR_WIDTH)
            last_py = WIN_HEIGHT * (1.0 - last_mapped_y / VR_HEIGHT)
            canvas.create_line(last_px, last_py, event.x, event.y, 
                             fill=current_color_base, width=line_width)
        
        # Add point to current stroke
        mapped_x, mapped_y = map_to_vr_coords(event.x, event.y)
        current_stroke.append((mapped_x, mapped_y))
        
        update_debug_display(event.x, event.y, mapped_x, mapped_y)

def get_visual_line_width():
    """Get visual line width based on current thickness setting"""
    if current_thickness == "Thick":
        return 4
    elif current_thickness == "Tight":
        return 1
    else:  # Should not happen, but fallback
        return 2

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
    format_text = f"Format: x1,y1 x2,y2 x3,y3 ... | Current: {get_current_color_code()}"
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
    stroke_colors.clear()
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
    stroke_colors.clear()
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
    global canvas, current_color_base, current_thickness
    root = tk.Tk()
    root.title(f"VR Coordinate Server - Enhanced Stroke Collection ({VR_WIDTH}x{VR_HEIGHT})")
    root.geometry(f"{WIN_WIDTH}x{WIN_HEIGHT+160}")

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

    # === Color and thickness selection section ===
    control_frame = tk.Frame(root)
    control_frame.pack(fill=tk.X, pady=5)
    
    # Color selection frame
    color_frame = tk.Frame(control_frame)
    color_frame.pack(side=tk.LEFT, padx=10)
    
    tk.Label(color_frame, text="Color:", font=("Arial", 10, "bold")).pack()
    color_buttons_frame = tk.Frame(color_frame)
    color_buttons_frame.pack()

    selected_color_label = tk.Label(color_frame, text="Selected Color:", fg="black")
    selected_color_label.pack()

    selected_color_box = tk.Canvas(color_frame, width=60, height=25, highlightthickness=1, highlightbackground="black")
    selected_color_box.pack()
    selected_color_box_rect = selected_color_box.create_rectangle(0, 0, 60, 25, fill=current_color_base)

    def set_color(hex_code):
        global current_color_base
        current_color_base = hex_code
        selected_color_box.itemconfig(selected_color_box_rect, fill=hex_code)
        selected_color_label.config(text=f"Color: {hex_code}")
        thickness_label.config(text=f"Mode: {current_thickness} ({get_current_color_code()})")
        update_status_display()

    # Create color buttons (only base colors for visual selection)
    for hex_code in BASE_COLORS.keys():
        btn = tk.Button(color_buttons_frame, bg=hex_code, width=3, height=1,
                       command=lambda c=hex_code: set_color(c))
        btn.pack(side=tk.LEFT, padx=1)

    # Thickness selection frame
    thickness_frame = tk.Frame(control_frame)
    thickness_frame.pack(side=tk.LEFT, padx=20)
    
    tk.Label(thickness_frame, text="Thickness Mode:", font=("Arial", 10, "bold")).pack()
    
    thickness_var = tk.StringVar(value=current_thickness)
    
    def on_thickness_change():
        global current_thickness
        current_thickness = thickness_var.get()
        thickness_label.config(text=f"Mode: {current_thickness} ({get_current_color_code()})")
        update_status_display()
    
    for thickness_name in THICKNESS_OPTIONS.keys():
        rb = tk.Radiobutton(thickness_frame, text=thickness_name, 
                           variable=thickness_var, value=thickness_name,
                           command=on_thickness_change)
        rb.pack(anchor="w")
    
    thickness_label = tk.Label(thickness_frame, text=f"Mode: {current_thickness} ({get_current_color_code()})", 
                              fg="blue", font=("Arial", 9))
    thickness_label.pack()

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
                         text=f"VR System: TL({VR_WIDTH},{VR_HEIGHT}) → BR(0,0) | Enhanced Thickness Support", 
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
    print("=== TCP Mouse Pointer Server - Enhanced VR Coordinate Simulator ===")
    print(f"VR coordinate system: {VR_WIDTH}x{VR_HEIGHT}")
    print(f"Top-left: ({VR_WIDTH}, {VR_HEIGHT})")
    print(f"Top-right: (0, {VR_HEIGHT})")
    print(f"Bottom-left: ({VR_WIDTH}, 0)")
    print(f"Bottom-right: (0, 0)")
    print("Message format: x1,y1 x2,y2 x3,y3 x4,y4 ... (one stroke per message)")
    print("Enhanced features: Thick/Tight color variants only")
    print("Transmission: On-demand via 'Send Strokes' button")
    print("=================================================================")
    
    threading.Thread(target=tcp_server, daemon=True).start()
    start_gui()