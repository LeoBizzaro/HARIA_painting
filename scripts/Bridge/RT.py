#!/usr/bin/env python3
import socket
import threading
import rospy
import sys
import tty
import termios
import actionlib
import time
import math
from geometry_msgs.msg import PoseStamped
from franka_gripper.msg import MoveAction, MoveGoal, GraspAction, GraspGoal

# === Parametri TCP ===
SERVER_IP = "127.0.0.1" # 127.0.0.1   192.168.1.111
SERVER_PORT = 5005

# === WORKSPACE CONFIGURATION ===
# Paper sizes in meters (width x height)
PAPER_SIZES = {
    "A2": (0.420, 0.594),
    "A3": (0.253, 0.382),
    "A4": (0.210, 0.297),
    "A5": (0.148, 0.210)
}

# Select your paper size here - change this to switch workspace
SELECTED_PAPER = "A3"
PAPER_WIDTH, PAPER_HEIGHT = PAPER_SIZES[SELECTED_PAPER]

# Robot workspace center position
WORKSPACE_CENTER_X = 0.5
WORKSPACE_CENTER_Y = 0.0

# COORDINATE SYSTEM ORIENTATION FLAGS
# Based on the reported issues, it seems X needs to be flipped
FLIP_X = True   # Set to True because right GUI -> left robot (needs flip)
FLIP_Y = True  # Y seems correct based on description

# Calculate workspace boundaries based on paper size and center
# Since we're swapping R2_X↔FR3_Y and R2_Y↔FR3_X:
# FR3_X direction (forward/back) corresponds to paper HEIGHT
# FR3_Y direction (left/right) corresponds to paper WIDTH
WORKSPACE_X_MIN = WORKSPACE_CENTER_X - PAPER_WIDTH / 2   # FR3_X uses paper width (21cm)
WORKSPACE_X_MAX = WORKSPACE_CENTER_X + PAPER_WIDTH / 2
WORKSPACE_Y_MIN = WORKSPACE_CENTER_Y - PAPER_HEIGHT / 2  # FR3_Y uses paper height (29.7cm)
WORKSPACE_Y_MAX = WORKSPACE_CENTER_Y + PAPER_HEIGHT / 2

# === Altezza e orientamento ===
MIN_Z_HEIGHT = 0.09
MAX_Z_HEIGHT = 0.25
Z_ACTIVE = 0.116  # This will be adjustable with +/- keys
Z_IDLE = 0.13
CURRENT_Z_HEIGHT = Z_IDLE
Z_INCREMENT = 0.002  # Fine increment for Z_ACTIVE adjustments

# Z-offset for safe approach from above (in meters)
Z_APPROACH_OFFSET = 0.2  # 4cm above the pencil position

FIXED_ORIENTATION = {
    "x": 1.0,
    "y": 0.0,
    "z": 0.0,
    "w": 0.0
}

# === Stato gripper ===
gripper_closed = False
gripper_available = False

# === PENCIL/COLOR SYSTEM ===
# Base pencil pickup position (you can adjust these coordinates)
# PENCIL_BASE_X = 0.32  # Base X coordinate for pencil pickup
# PENCIL_BASE_Y_LEFT = -0.08  # Y coordinate for pencil pickup (same for all colors)
# PENCIL_BASE_Y_RIGHT = 0.24
# PENCIL_BASE_Z = 0.11 # Z coordinate for pencil pickup (same for all colors)
# PENCIL_Y_OFFSET = 0.08  # 5cm offset between pencil positions

# # Pencil positions for each color
# PENCIL_POSITIONS = {
#     1: {"x": PENCIL_BASE_X, "y": PENCIL_BASE_Y_LEFT - 4*PENCIL_Y_OFFSET, "z": PENCIL_BASE_Z, "name": "Color 1"},
#     2: {"x": PENCIL_BASE_X, "y": PENCIL_BASE_Y_LEFT - 3*PENCIL_Y_OFFSET, "z": PENCIL_BASE_Z, "name": "Color 2"},
#     3: {"x": PENCIL_BASE_X, "y": PENCIL_BASE_Y_LEFT - 2*PENCIL_Y_OFFSET, "z": PENCIL_BASE_Z, "name": "Color 3"},
#     4: {"x": PENCIL_BASE_X, "y": PENCIL_BASE_Y_LEFT - PENCIL_Y_OFFSET, "z": PENCIL_BASE_Z, "name": "Color 4"},
#     5: {"x": PENCIL_BASE_X, "y": PENCIL_BASE_Y_LEFT, "z": PENCIL_BASE_Z, "name": "Color 5"},
#     6: {"x": PENCIL_BASE_X, "y": PENCIL_BASE_Y_RIGHT, "z": PENCIL_BASE_Z, "name": "Color 6"},
#     7: {"x": PENCIL_BASE_X, "y": PENCIL_BASE_Y_RIGHT + PENCIL_Y_OFFSET, "z": PENCIL_BASE_Z, "name": "Color 7"},
#     8: {"x": PENCIL_BASE_X, "y": PENCIL_BASE_Y_RIGHT + 2*PENCIL_Y_OFFSET, "z": PENCIL_BASE_Z, "name": "Color 8"},
#     9: {"x": PENCIL_BASE_X, "y": PENCIL_BASE_Y_RIGHT +  3*PENCIL_Y_OFFSET, "z": PENCIL_BASE_Z, "name": "Color 9"},
#     10: {"x": PENCIL_BASE_X, "y": PENCIL_BASE_Y_RIGHT + 4*PENCIL_Y_OFFSET, "z": PENCIL_BASE_Z, "name": "Color 10"}
# }

# === PENCIL/COLOR SYSTEM ===
# Base pencil pickup position
PENCIL_BASE_X = 0.326        # Starting X coordinate for pencil pickup
PENCIL_BASE_Y_LEFT = -0.32  # Y coordinate for left pencils
PENCIL_BASE_Y_RIGHT = 0.32  # Y coordinate for right pencils
PENCIL_BASE_Z = 0.125        # Z coordinate for pencil pickup
PENCIL_X_OFFSET = 0.06      # 8cm offset along X between pencils

# Pencil positions for each color
PENCIL_POSITIONS = {
    1: {"x": PENCIL_BASE_X + 0*PENCIL_X_OFFSET, "y": PENCIL_BASE_Y_LEFT, "z": PENCIL_BASE_Z, "name": "Color 1"},
    2: {"x": PENCIL_BASE_X + 1*PENCIL_X_OFFSET, "y": PENCIL_BASE_Y_LEFT, "z": PENCIL_BASE_Z, "name": "Color 2"},
    3: {"x": PENCIL_BASE_X + 2*PENCIL_X_OFFSET, "y": PENCIL_BASE_Y_LEFT, "z": PENCIL_BASE_Z, "name": "Color 3"},
    4: {"x": PENCIL_BASE_X + 3*PENCIL_X_OFFSET, "y": PENCIL_BASE_Y_LEFT, "z": PENCIL_BASE_Z, "name": "Color 4"},
    5: {"x": PENCIL_BASE_X + 4*PENCIL_X_OFFSET, "y": PENCIL_BASE_Y_LEFT, "z": PENCIL_BASE_Z, "name": "Color 5"},
    6: {"x": PENCIL_BASE_X + 0*PENCIL_X_OFFSET, "y": PENCIL_BASE_Y_RIGHT, "z": PENCIL_BASE_Z, "name": "Color 6"},
    7: {"x": PENCIL_BASE_X + 1*PENCIL_X_OFFSET, "y": PENCIL_BASE_Y_RIGHT, "z": PENCIL_BASE_Z, "name": "Color 7"},
    8: {"x": PENCIL_BASE_X + 2*PENCIL_X_OFFSET, "y": PENCIL_BASE_Y_RIGHT, "z": PENCIL_BASE_Z, "name": "Color 8"},
    9: {"x": PENCIL_BASE_X + 3*PENCIL_X_OFFSET, "y": PENCIL_BASE_Y_RIGHT, "z": PENCIL_BASE_Z, "name": "Color 9"},
    10: {"x": PENCIL_BASE_X + 4*PENCIL_X_OFFSET, "y": PENCIL_BASE_Y_RIGHT, "z": PENCIL_BASE_Z, "name": "Color 10"}
}


COLOR_MAP = {
    "#7A3D28": 1,  # Brown
    "#D82929": 2,  # Red
    "#E86E09": 3,  # Orange
    "#DBC416": 4,  # Yellow
    "#16160F": 5,  # Black
    "#49C893": 6,  # Light Green
    "#02704D": 7,  # Dark Green
    "#0D2875": 8,  # Dark Blue
    "#0295D5": 9,  # Light Blue
    "#C0BDAE": 10,  # White
}

current_pencil = None  # Currently selected pencil (1, 2, or 3)
pencil_ready = False   # True when ready to draw (simplified - always True after pickup)

# === Stato robot ===
last_valid_x = WORKSPACE_CENTER_X
last_valid_y = WORKSPACE_CENTER_Y
last_tcp_time = 0

pub = None

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def clamp_to_workspace_bounds(x, y):
    """
    Clamp coordinates to the defined workspace boundaries
    """
    x_clamped = max(WORKSPACE_X_MIN, min(x, WORKSPACE_X_MAX))
    y_clamped = max(WORKSPACE_Y_MIN, min(y, WORKSPACE_Y_MAX))
    return x_clamped, y_clamped

def map_normalized_to_workspace(norm_x, norm_y):
    """
    Map normalized canvas coordinates (0-1) to FR3 workspace coordinates
    
    Canvas coordinate system:
    - (0, 0) = bottom-right
    - (1400, 1000) = top-left  
    
    FR3 frame (viewed from above): X forward, Y left, Z up
    
    SWAPPED MAPPING to fix coordinate rotation:
    - Canvas X → FR3 Y (swap X and Y axes)
    - Canvas Y → FR3 X (swap X and Y axes)
    """
    # Apply coordinate flips if needed
    if FLIP_X:
        norm_x = 1.0 - norm_x
    if FLIP_Y:
        norm_y = 1.0 - norm_y
    
    # SWAPPED MAPPING: Canvas X→FR3 Y, Canvas Y→FR3 X
    real_x = WORKSPACE_X_MIN + norm_y * PAPER_WIDTH   # Canvas_Y → FR3_X
    real_y = WORKSPACE_Y_MIN + norm_x * PAPER_HEIGHT  # Canvas_X → FR3_Y
    
    rospy.logdebug(f"Canvas→FR3 SWAPPED mapping: Canvas({norm_x:.3f},{norm_y:.3f}) → FR3({real_x:.3f},{real_y:.3f})")
    
    return real_x, real_y

def publish_pose(x, y, z):
    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "fr3_link0"
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z
    msg.pose.orientation.x = FIXED_ORIENTATION['x']
    msg.pose.orientation.y = FIXED_ORIENTATION['y']
    msg.pose.orientation.z = FIXED_ORIENTATION['z']
    msg.pose.orientation.w = FIXED_ORIENTATION['w']
    pub.publish(msg)

def perform_gradual_movement(start_x, start_y, start_z, end_x, end_y, end_z, steps=10, delay=0.5):
    """
    Perform a gradual movement from start position to end position in multiple steps
    """
    rospy.loginfo(f"Starting gradual movement: ({start_x}, {start_y}, {start_z}) -> ({end_x}, {end_y}, {end_z})")
    
    for step in range(1, steps + 1):
        # Calculate intermediate position
        progress = step / steps
        x = start_x + (end_x - start_x) * progress
        y = start_y + (end_y - start_y) * progress
        z = start_z + (end_z - start_z) * progress
        
        # Move to intermediate position
        publish_pose(x, y, z)
        rospy.loginfo(f"Step {step}/{steps}: Moving to ({x:.3f}, {y:.3f}, {z:.3f})")
        time.sleep(delay)

def return_pencil_to_position(move_client, grasp_client, pencil_number):
    """
    Return the current pencil to its designated position
    Uses Z-offset approach for safe movement
    SIMPLIFIED: Always assumes success
    """
    global gripper_closed, current_pencil, pencil_ready, CURRENT_Z_HEIGHT
    
    if pencil_number not in PENCIL_POSITIONS:
        rospy.logwarn(f"Invalid pencil number: {pencil_number}")
        return
    
    pencil_pos = PENCIL_POSITIONS[pencil_number]
    approach_z = pencil_pos['z'] + Z_APPROACH_OFFSET
    
    rospy.loginfo(f"Returning {pencil_pos['name']} to position with Z-offset approach...")
    
    # Step 1: Move to approach position (above pencil position)
    rospy.loginfo(f"Moving to approach position above {pencil_pos['name']}...")
    perform_gradual_movement(
        last_valid_x, last_valid_y, CURRENT_Z_HEIGHT,
        pencil_pos['x'], pencil_pos['y'], approach_z,
        steps=8, delay=0.6
    )
    
    # Step 2: Descend to pencil return position
    rospy.loginfo(f"Descending to {pencil_pos['name']} return position...")
    perform_gradual_movement(
        pencil_pos['x'], pencil_pos['y'], approach_z,
        pencil_pos['x'], pencil_pos['y'], pencil_pos['z'],
        steps=4, delay=0.4
    )
    
    # Step 3: Open gripper to release pencil
    rospy.loginfo("Releasing pencil...")
    if gripper_available:
        move_goal = MoveGoal(width=0.05, speed=0.1)
        move_client.send_goal(move_goal)
        move_client.wait_for_result()
    gripper_closed = False
    time.sleep(1.0)
    
    # Step 4: Move back up to approach position after releasing
    rospy.loginfo("Lifting from pencil position to approach position...")
    perform_gradual_movement(
        pencil_pos['x'], pencil_pos['y'], pencil_pos['z'],
        pencil_pos['x'], pencil_pos['y'], approach_z,
        steps=4, delay=0.4
    )

    # Step 5: Move gradually to idle position (e.g., last_valid_x, last_valid_y, Z_IDLE)
    rospy.loginfo("Returning to idle position...")
    perform_gradual_movement(
        pencil_pos['x'], pencil_pos['y'], approach_z,
        last_valid_x, last_valid_y, Z_IDLE,
        steps=8, delay=0.6
    )

    # Update current Z height
    CURRENT_Z_HEIGHT = Z_IDLE

    
    # SIMPLIFIED: Always consider it successful
    current_pencil = None
    pencil_ready = False
    
    rospy.loginfo(f"{pencil_pos['name']} returned successfully with Z-offset approach.")

def pickup_pencil_sequence(move_client, grasp_client, pencil_number):
    """
    Pick up a specific pencil/color
    Uses Z-offset approach for safe movement
    SIMPLIFIED: Always assumes successful pickup
    """
    global gripper_closed, current_pencil, pencil_ready, last_valid_x, last_valid_y, CURRENT_Z_HEIGHT
    
    if pencil_number not in PENCIL_POSITIONS:
        rospy.logwarn(f"Invalid pencil number: {pencil_number}")
        return
    
    pencil_pos = PENCIL_POSITIONS[pencil_number]
    approach_z = pencil_pos['z'] + Z_APPROACH_OFFSET
    
    # If we already have a pencil, return it first
    if current_pencil is not None and current_pencil != pencil_number:
        rospy.loginfo(f"Returning current pencil ({PENCIL_POSITIONS[current_pencil]['name']}) before picking up {pencil_pos['name']}")
        return_pencil_to_position(move_client, grasp_client, current_pencil)
    
    # If we're trying to pick up the same pencil we already have, just continue
    if current_pencil == pencil_number and pencil_ready:
        rospy.loginfo(f"{pencil_pos['name']} is already selected and ready.")
        return
    
    rospy.loginfo(f"Starting pickup sequence for {pencil_pos['name']} with Z-offset approach...")
    
    # Step 1: Open gripper fully
    rospy.loginfo("Opening gripper...")
    if gripper_available:
        move_goal = MoveGoal(width=0.05, speed=0.1)
        move_client.send_goal(move_goal)
        move_client.wait_for_result()
    gripper_closed = False
    time.sleep(1.0)
    
    # Step 2: Move to approach position (above pencil)
    rospy.loginfo(f"Moving to approach position above {pencil_pos['name']}...")
    perform_gradual_movement(
        last_valid_x, last_valid_y, CURRENT_Z_HEIGHT,
        pencil_pos['x'], pencil_pos['y'], approach_z,
        steps=8, delay=0.6
    )
    
    # Step 3: Descend to pencil pickup position
    rospy.loginfo(f"Descending to {pencil_pos['name']} pickup position...")
    perform_gradual_movement(
        pencil_pos['x'], pencil_pos['y'], approach_z,
        pencil_pos['x'], pencil_pos['y'], pencil_pos['z'],
        steps=4, delay=0.4
    )
    
    # Step 4: Wait briefly for positioning
    rospy.loginfo("Positioning for pickup...")
    time.sleep(1.0)
    
    # Step 5: Close gripper gently to grasp pencil
    rospy.loginfo(f"Grasping {pencil_pos['name']}...")
    if gripper_available:
        grasp_goal = GraspGoal(width=0.024, speed=0.05, force=10.0)
        grasp_goal.epsilon.inner = 0.002
        grasp_goal.epsilon.outer = 0.008
        grasp_client.send_goal(grasp_goal)
        grasp_client.wait_for_result()
    gripper_closed = True
    time.sleep(1.0)
    
    # Step 6: Move back up to approach position with pencil
    rospy.loginfo("Lifting pencil to approach position...")
    perform_gradual_movement(
        pencil_pos['x'], pencil_pos['y'], pencil_pos['z'],  # Current position
        pencil_pos['x'], pencil_pos['y'], approach_z,       # Target position
        steps=4, delay=0.5  # Slow and careful movement
    )
    
    # Step 7: Move to drawing position gradually
    rospy.loginfo("Moving to drawing position...")
    perform_gradual_movement(
        pencil_pos['x'], pencil_pos['y'], approach_z,
        last_valid_x, last_valid_y, Z_IDLE,
        steps=8, delay=0.6
    )
    
    # SIMPLIFIED: Always assume pickup was successful
    CURRENT_Z_HEIGHT = Z_IDLE
    current_pencil = pencil_number
    pencil_ready = True  # Always set to True after pickup sequence
    
    rospy.loginfo(f"{pencil_pos['name']} pickup complete with Z-offset approach. Robot is ready for drawing.")
    rospy.loginfo(f"PENCIL_READY = {pencil_ready} (simplified - always True after pickup)")


def keyboard_listener():
    global CURRENT_Z_HEIGHT, gripper_closed, gripper_available, pencil_ready, Z_ACTIVE
    global current_pencil

    move_client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
    grasp_client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)

    try:
        rospy.loginfo("Checking gripper servers...")
        if move_client.wait_for_server(rospy.Duration(2.0)) and grasp_client.wait_for_server(rospy.Duration(2.0)):
            gripper_available = True
            rospy.loginfo("Gripper servers connected.")
        else:
            rospy.logwarn("Gripper servers unavailable. Will proceed without gripper control.")
    except Exception as e:
        rospy.logwarn(f"Gripper connection failed: {e}. Will proceed without gripper control.")

    rospy.loginfo(f"Keyboard ready: +/- = adjust active height, O/C = gripper")
    rospy.loginfo(f"Colors: Now selected via TCP color codes (e.g., #FF0000)")
    rospy.loginfo(f"R = return current pencil, I = system info")
    rospy.loginfo(f"Current workspace: {SELECTED_PAPER} paper ({PAPER_WIDTH}x{PAPER_HEIGHT}m)")
    rospy.loginfo(f"SIMPLIFIED MODE: Pickup always succeeds, pencil_ready always True after pickup")
    rospy.loginfo(f"Available colors: {list(COLOR_MAP.keys())}")
    
    while not rospy.is_shutdown():
        c = getch()
        # Rimuovi la selezione manuale dei colori 1,2,3
        if c in ['r', 'R'] and current_pencil is not None:
            # Return current pencil to its position
            return_pencil_to_position(move_client, grasp_client, current_pencil)
        elif c in ['+', '=']:
            # Adjust Z_ACTIVE with fine control
            Z_ACTIVE = min(MAX_Z_HEIGHT, Z_ACTIVE + Z_INCREMENT)
            rospy.loginfo(f"Z_ACTIVE height adjusted to: {Z_ACTIVE:.3f}")
            # If pencil is active and in drawing mode, update the current position
            if pencil_ready and (time.time() - last_tcp_time <= 2.0):
                publish_pose(last_valid_x, last_valid_y, Z_ACTIVE)
        elif c in ['-', '_']:
            # Adjust Z_ACTIVE with fine control
            Z_ACTIVE = max(MIN_Z_HEIGHT, Z_ACTIVE - Z_INCREMENT)
            rospy.loginfo(f"Z_ACTIVE height adjusted to: {Z_ACTIVE:.3f}")
            # If pencil is active and in drawing mode, update the current position
            if pencil_ready and (time.time() - last_tcp_time <= 2.0):
                publish_pose(last_valid_x, last_valid_y, Z_ACTIVE)
        elif c in ['o', 'O'] and gripper_available:
            goal = MoveGoal(width=0.08, speed=0.1)
            move_client.send_goal(goal)
            move_client.wait_for_result()
            gripper_closed = False
        elif c in ['c', 'C'] and gripper_available:
            goal = GraspGoal(width=0.01, speed=0.05, force=10.0)
            goal.epsilon.inner = 0.003
            goal.epsilon.outer = 0.003
            grasp_client.send_goal(goal)
            grasp_client.wait_for_result()
            gripper_closed = True
        elif c in ['q', 'Q']:
            # Return pencil before quitting if one is held
            if current_pencil is not None:
                rospy.loginfo("Returning pencil before shutdown...")
                return_pencil_to_position(move_client, grasp_client, current_pencil)
            rospy.signal_shutdown("Keyboard quit")
            break

def tcp_receiver():
    global last_valid_x, last_valid_y, last_tcp_time

    # Inizializza i client gripper per la gestione dei colori
    move_client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
    grasp_client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)
    
    # Attendi i server gripper
    try:
        if move_client.wait_for_server(rospy.Duration(2.0)) and grasp_client.wait_for_server(rospy.Duration(2.0)):
            rospy.loginfo("Gripper clients ready for color changes.")
        else:
            rospy.logwarn("Gripper servers not available for color changes.")
    except Exception as e:
        rospy.logwarn(f"Gripper setup failed: {e}")

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        rospy.loginfo(f"Connecting to TCP server at {SERVER_IP}:{SERVER_PORT}...")
        sock.connect((SERVER_IP, SERVER_PORT))
        rospy.loginfo("Connected. Receiving VR coordinates and color codes.")
        rospy.loginfo("Supported formats: 'x,y' (coordinates) or '#RRGGBB' (color code only)")
        rospy.loginfo(f"Available colors: {list(COLOR_MAP.keys())}")
        
        # Add socket timeout to prevent indefinite blocking
        sock.settimeout(1.0)
        
        buffer = b""
        message_count = 0
        error_count = 0
        color_changes = 0
        coordinate_messages = 0
        last_debug_time = time.time()
        
        while not rospy.is_shutdown():
            try:
                data = sock.recv(1024)
                if not data:
                    rospy.logwarn("Connection closed by server.")
                    break

                buffer += data
                
                # Process all complete messages in buffer
                while b"\n" in buffer:
                    line, buffer = buffer.split(b"\n", 1)
                    message_count += 1
                    
                    try:
                        text = line.decode().strip()
                        if not text:  # Skip empty lines
                            continue
                            
                        # Parse del messaggio TCP
                        vr_x, vr_y, color_code = parse_tcp_message(text)
                        
                        # Handle color-only messages
                        if color_code is not None and vr_x is None and vr_y is None:
                            rospy.loginfo(f"Received color-only message: {color_code}")
                            handle_color_change(color_code, move_client, grasp_client)
                            color_changes += 1
                            continue  # Skip coordinate processing for color-only messages
                        
                        # Handle coordinate messages
                        if vr_x is not None and vr_y is not None:
                            coordinate_messages += 1
                            
                            # Handle color change if present with coordinates
                            if color_code is not None:
                                handle_color_change(color_code, move_client, grasp_client)
                                color_changes += 1
                            
                            # Log raw received data for debugging
                            if coordinate_messages % 120 == 0:  # Log every 2 seconds at 60Hz
                                rospy.loginfo(f"TCP coords: '{text}' -> ({vr_x},{vr_y}) color({color_code}) | pencil_ready={pencil_ready}")
                            
                            # Convert VR coordinates (0-1400 for X, 0-1000 for Y) to normalized (0-1)
                            # VR system: (0,0)=bottom-right, (1400,1000)=top-left
                            norm_x = vr_x / 1400.0
                            norm_y = vr_y / 1000.0
                            
                            # Validate coordinate ranges
                            if not (0.0 <= norm_x <= 1.0 and 0.0 <= norm_y <= 1.0):
                                if coordinate_messages % 60 == 0:  # Don't spam warnings
                                    rospy.logwarn(f"VR coordinates out of range: VR({vr_x}, {vr_y}) -> norm({norm_x:.3f}, {norm_y:.3f})")
                                continue
                            
                            # Convert VR coordinate system to standard (0,0)=bottom-left
                            # VR: (0,0)=bottom-right -> Standard: (1,0)=bottom-right
                            # VR: (1400,1000)=top-left -> Standard: (0,1)=top-left  
                            standard_x = norm_x  # X mapping
                            standard_y = norm_y  # Y mapping
                            
                            # Map standard normalized coordinates to workspace
                            raw_x, raw_y = map_normalized_to_workspace(standard_x, standard_y)
                            raw_x, raw_y = clamp_to_workspace_bounds(raw_x, raw_y)
                            
                            # Log mapping every 2 seconds
                            if coordinate_messages % 120 == 0:
                                rospy.loginfo(f"VR→FR3: VR({vr_x},{vr_y}) -> std({standard_x:.3f},{standard_y:.3f}) -> "
                                            f"FR3({raw_x:.3f},{raw_y:.3f}) | pencil_ready={pencil_ready}")
                            
                            # Update position directly (no filtering)
                            last_valid_x = raw_x
                            last_valid_y = raw_y
                            last_tcp_time = time.time()
                            
                            # Robot moves if pencil_ready is True
                            if pencil_ready:
                                publish_pose(raw_x, raw_y, Z_ACTIVE)
                            else:
                                # Log when coordinates are received but pencil is not ready
                                if coordinate_messages % 60 == 0:  # Don't spam this message
                                    rospy.loginfo(f"Coordinates received but pencil not ready. Use color codes to select pencil.")
                        else:
                            # Invalid message format
                            rospy.logwarn(f"Invalid TCP message format: '{text}' - Expected 'x,y' or '#RRGGBB'")
                            error_count += 1
                            
                    except Exception as e:
                        rospy.logwarn(f"Parsing error: {e} for line: '{text}'")
                        error_count += 1
                
                # Debug output every 2 seconds
                current_time = time.time()
                if current_time - last_debug_time >= 2.0:
                    rospy.loginfo(f"TCP Stats: {message_count} total, {coordinate_messages} coords, "
                                f"{color_changes} color changes, {error_count} errors | pencil_ready={pencil_ready}")
                    last_debug_time = current_time
                    
            except socket.timeout:
                # Timeout is normal, just continue
                continue
            except Exception as e:
                rospy.logerr(f"TCP receive error: {e}")
                break
                
    except Exception as e:
        rospy.logerr(f"TCP connection failed: {e}")
    finally:
        sock.close()
        rospy.loginfo("TCP socket closed.")


def parse_tcp_message(message):
    """
    Parsa il messaggio TCP per estrarre coordinate o codice colore.
    Formati supportati:
    - "x,y" (solo coordinate)
    - "#RRGGBB" (solo colore)
    - "x,y #RRGGBB" (coordinate + colore)
    
    Returns:
        tuple: (x, y, color_code) where any can be None if not present
    """
    message = message.strip()
    
    if not message:
        return None, None, None

    # Check if message contains both coordinates and color
    parts = message.split()
    color_code = None
    coord_part = message
    
    # Look for color code in the message
    for part in parts:
        if part.startswith('#') and len(part) == 7:
            color_code = part.upper()
            # Remove color code from coordinate part
            coord_part = message.replace(part, '').strip()
            break
    
    # If only color code (no coordinates)
    if color_code and not coord_part:
        return None, None, color_code
    
    # Parse coordinates if present
    if coord_part and ',' in coord_part:
        coord_parts = coord_part.split(',')
        if len(coord_parts) == 2:
            try:
                x = float(coord_parts[0].strip())
                y = float(coord_parts[1].strip())
                return x, y, color_code
            except ValueError:
                return None, None, color_code if color_code else None
    
    # If only color code without coordinates
    if color_code and not coord_part:
        return None, None, color_code
    
    # If message starts with # and is 7 chars (pure color code)
    if message.startswith('#') and len(message) == 7:
        return None, None, message.upper()
    
    return None, None, None


def handle_color_change(color_code, move_client, grasp_client):
    """
    Gestisce il cambio di colore basato sul codice ricevuto via TCP.
    """
    global current_pencil, pencil_ready
    
    if color_code in COLOR_MAP:
        new_pencil = COLOR_MAP[color_code]
        
        # Se è già selezionato questo colore, non fare nulla
        if current_pencil == new_pencil:
            rospy.loginfo(f"Color {color_code} already active (Pencil {new_pencil})")
            return
        
        rospy.loginfo(f"Color change requested: {color_code} -> Pencil {new_pencil}")
        
        # Se c'è già una matita, restituiscila prima
        if current_pencil is not None:
            rospy.loginfo(f"Returning current pencil {current_pencil} before switching to {new_pencil}")
            return_pencil_to_position(move_client, grasp_client, current_pencil)
        
        # Prendi la nuova matita
        pickup_pencil_sequence(move_client, grasp_client, new_pencil)
        rospy.loginfo(f"Successfully switched to color {color_code} (Pencil {new_pencil})")
    else:
        rospy.logwarn(f"Unknown color code received: {color_code}")
        rospy.loginfo(f"Available colors: {list(COLOR_MAP.keys())}")


def idle_monitor():
    global CURRENT_Z_HEIGHT
    rate = rospy.Rate(2)  # 2 Hz
    already_idle = False  # Flag to track if we've already moved to idle position
    
    while not rospy.is_shutdown():
        if pencil_ready and (time.time() - last_tcp_time > 2.0):
            # Only publish once when transitioning to idle
            if not already_idle:
                CURRENT_Z_HEIGHT = Z_IDLE
                publish_pose(last_valid_x, last_valid_y, CURRENT_Z_HEIGHT)
                rospy.loginfo(f"Pencil moved to idle position: Z={Z_IDLE}")
                already_idle = True
        else:
            # Reset the flag when we're receiving coordinates again
            if already_idle and (time.time() - last_tcp_time <= 2.0):
                already_idle = False
                rospy.logdebug("Pencil back to active drawing mode")
        
        rate.sleep()

def main():
    global pub
    rospy.init_node("tcp_drawing_homepos_no_filter")
    pub = rospy.Publisher("/demo/pose_final", PoseStamped, queue_size=10)
    
    # Log workspace configuration at startup
    rospy.loginfo(f"=== WORKSPACE CONFIGURATION ===")
    rospy.loginfo(f"Paper size: {SELECTED_PAPER}")
    rospy.loginfo(f"Dimensions: {PAPER_WIDTH}m x {PAPER_HEIGHT}m")
    rospy.loginfo(f"Center: ({WORKSPACE_CENTER_X}, {WORKSPACE_CENTER_Y})")
    rospy.loginfo(f"FR3 X range: {WORKSPACE_X_MIN:.3f} to {WORKSPACE_X_MAX:.3f}")
    rospy.loginfo(f"FR3 Y range: {WORKSPACE_Y_MIN:.3f} to {WORKSPACE_Y_MAX:.3f}")
    rospy.loginfo(f"Coordinate mapping: R2_X→FR3_Y, R2_Y→FR3_X")
    rospy.loginfo(f"Coordinate flips: X={FLIP_X}, Y={FLIP_Y}")
    rospy.loginfo(f"VR Safety Filtering: DISABLED")
    rospy.loginfo(f"Color Selection: TCP-based with {len(COLOR_MAP)} colors")
    rospy.loginfo(f"Available colors: {list(COLOR_MAP.keys())}")
    rospy.loginfo(f"===============================")
    
    threading.Thread(target=keyboard_listener, daemon=True).start()
    threading.Thread(target=tcp_receiver, daemon=True).start()
    threading.Thread(target=idle_monitor, daemon=True).start()
    
    # Keep the node alive
    rospy.spin()

if __name__ == "__main__":
    main()