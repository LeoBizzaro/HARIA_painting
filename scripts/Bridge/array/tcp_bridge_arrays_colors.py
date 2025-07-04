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
SERVER_IP = "127.0.0.1" #127.0.0.1  192.168.1.111
SERVER_PORT = 5005

# === WORKSPACE CONFIGURATION ===
# Paper sizes in meters (width x height)
PAPER_SIZES = {
    "A2": (0.420, 0.594),
    "A3": (0.297, 0.420),
    "A4": (0.210, 0.297),
    "A5": (0.148, 0.210)
}

# Select your paper size here - change this to switch workspace
SELECTED_PAPER = "A4"
PAPER_WIDTH, PAPER_HEIGHT = PAPER_SIZES[SELECTED_PAPER]

# Robot workspace center position
WORKSPACE_CENTER_X = 0.5
WORKSPACE_CENTER_Y = 0.0

# COORDINATE SYSTEM ORIENTATION FLAGS
# Based on the reported issues, it seems X needs to be flipped
FLIP_X = True   # Set to True because right GUI -> left robot (needs flip) (True)
FLIP_Y = True  # Y seems correct based on description (False)

# Calculate workspace boundaries for HORIZONTAL A4 layout
# FR3_X direction (forward/back) should correspond to canvas WIDTH (1400)
# FR3_Y direction (left/right) should correspond to canvas HEIGHT (1000)
WORKSPACE_X_MIN = WORKSPACE_CENTER_X - PAPER_WIDTH / 2   # FR3_X uses paper width (21cm)
WORKSPACE_X_MAX = WORKSPACE_CENTER_X + PAPER_WIDTH / 2
WORKSPACE_Y_MIN = WORKSPACE_CENTER_Y - PAPER_HEIGHT / 2  # FR3_Y uses paper height (29.7cm)  
WORKSPACE_Y_MAX = WORKSPACE_CENTER_Y + PAPER_HEIGHT / 2

# === Altezza e orientamento ===
MIN_Z_HEIGHT = 0.125
MAX_Z_HEIGHT = 0.50
Z_ACTIVE = 0.178  # This will be adjustable with +/- keys
Z_IDLE = 0.190
Z_LIFT = 0.200  # Height to lift between arrays
CURRENT_Z_HEIGHT = Z_IDLE
Z_INCREMENT = 0.002  # Fine increment for Z_ACTIVE adjustments

# Z-offset for safe approach from above (in meters)
Z_APPROACH_OFFSET = 0.04  # 4cm above the pencil position

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
PENCIL_BASE_X = 0.3  # Base X coordinate for pencil pickup
PENCIL_BASE_Y = 0.2  # Y coordinate for pencil pickup (same for all colors)
PENCIL_BASE_Z = 0.25 # Z coordinate for pencil pickup (same for all colors)
PENCIL_X_OFFSET = 0.05  # 5cm offset between pencil positions

# Pencil positions for each color
PENCIL_POSITIONS = {
    1: {"x": PENCIL_BASE_X, "y": PENCIL_BASE_Y, "z": PENCIL_BASE_Z, "name": "Color 1"},
    2: {"x": PENCIL_BASE_X + PENCIL_X_OFFSET, "y": PENCIL_BASE_Y, "z": PENCIL_BASE_Z, "name": "Color 2"},
    3: {"x": PENCIL_BASE_X + 2 * PENCIL_X_OFFSET, "y": PENCIL_BASE_Y, "z": PENCIL_BASE_Z, "name": "Color 3"},
    4: {"x": PENCIL_BASE_X + 3 * PENCIL_X_OFFSET, "y": PENCIL_BASE_Y, "z": PENCIL_BASE_Z, "name": "Color 4"},
    5: {"x": PENCIL_BASE_X + 4 * PENCIL_X_OFFSET, "y": PENCIL_BASE_Y, "z": PENCIL_BASE_Z, "name": "Color 5"},
    6: {"x": PENCIL_BASE_X + 5 * PENCIL_X_OFFSET, "y": PENCIL_BASE_Y, "z": PENCIL_BASE_Z, "name": "Color 6"},
    7: {"x": PENCIL_BASE_X + 6 * PENCIL_X_OFFSET, "y": PENCIL_BASE_Y, "z": PENCIL_BASE_Z, "name": "Color 7"},
    8: {"x": PENCIL_BASE_X + 7 * PENCIL_X_OFFSET, "y": PENCIL_BASE_Y, "z": PENCIL_BASE_Z, "name": "Color 8"}
}

current_pencil = None  # Currently selected pencil (1, 2, or 3)
pencil_ready = False   # True when ready to draw (simplified - always True after pickup)

# === Array processing state ===
current_array = []
processing_array = False
array_count = 0

# === Stato robot ===
last_valid_x = WORKSPACE_CENTER_X
last_valid_y = WORKSPACE_CENTER_Y

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
        move_goal = MoveGoal(width=0.08, speed=0.1)
        move_client.send_goal(move_goal)
        move_client.wait_for_result()
    gripper_closed = False
    time.sleep(1.0)
    
    # Step 4: Move back up to approach position after releasing
    rospy.loginfo("Moving up after pencil release...")
    publish_pose(pencil_pos['x'], pencil_pos['y'], approach_z)
    time.sleep(0.8)
    
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
        move_goal = MoveGoal(width=0.08, speed=0.1)
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
        grasp_goal = GraspGoal(width=0.019, speed=0.05, force=10.0)
        grasp_goal.epsilon.inner = 0.003
        grasp_goal.epsilon.outer = 0.003
        grasp_client.send_goal(grasp_goal)
        grasp_client.wait_for_result()
    gripper_closed = True
    time.sleep(1.0)
    
    # Step 6: Move back up to approach position with pencil
    rospy.loginfo("Lifting pencil to approach position...")
    publish_pose(pencil_pos['x'], pencil_pos['y'], approach_z)
    time.sleep(0.8)
    
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

def process_coordinate_array(coordinates):
    """
    Process an array of coordinates by moving through them sequentially
    """
    global last_valid_x, last_valid_y, CURRENT_Z_HEIGHT, processing_array, array_count
    
    if not pencil_ready:
        rospy.logwarn("Cannot process coordinate array: no pencil selected. Use keys 1/2/3 to select a pencil first.")
        return
    
    if len(coordinates) == 0:
        rospy.logwarn("Received empty coordinate array")
        return
    
    processing_array = True
    array_count += 1
    
    rospy.loginfo(f"=== PROCESSING ARRAY #{array_count} with {len(coordinates)} coordinates ===")
    
    # Step 1: Lift the end effector
    rospy.loginfo("Lifting end effector before array execution...")
    publish_pose(last_valid_x, last_valid_y, Z_LIFT)
    time.sleep(0.5)
    
    # Step 2: Move to the first coordinate of the array (at lift height)
    first_x, first_y = coordinates[0]
    rospy.loginfo(f"Moving to first coordinate: ({first_x:.3f}, {first_y:.3f}) at lift height")
    publish_pose(first_x, first_y, Z_LIFT)
    time.sleep(0.3)
    
    # Step 3: Lower to active drawing height
    rospy.loginfo("Lowering to drawing height...")
    publish_pose(first_x, first_y, Z_ACTIVE)
    time.sleep(0.3)
    
    # Step 4: Execute the array by moving through all coordinates
    rospy.loginfo("Executing coordinate array...")
    for i, (x, y) in enumerate(coordinates):
        publish_pose(x, y, Z_ACTIVE)
        last_valid_x = x
        last_valid_y = y
        
        # Small delay between points to ensure smooth movement
        time.sleep(0.05)  # 50ms delay - adjust as needed
        
        # Log progress periodically
        if (i + 1) % max(1, len(coordinates) // 10) == 0:
            progress = ((i + 1) / len(coordinates)) * 100
            rospy.loginfo(f"Array progress: {i+1}/{len(coordinates)} ({progress:.1f}%)")
    
    # Step 5: Lift the end effector after completing the array
    rospy.loginfo("Array complete. Lifting end effector...")
    publish_pose(last_valid_x, last_valid_y, Z_LIFT)
    time.sleep(0.3)
    
    # Update current height
    CURRENT_Z_HEIGHT = Z_LIFT
    
    processing_array = False
    rospy.loginfo(f"=== ARRAY #{array_count} COMPLETED ===")


def parse_coordinate_line(line):
    """
    Parse a line containing coordinate pairs separated by spaces
    Expected format: "x1,y1 x2,y2 x3,y3 ..."
    
    Canvas coordinate system:
    - Canvas range: X: 0-1400, Y: 0-1000
    - Let the FLIP_X and FLIP_Y flags handle coordinate transformation
    
    Returns list of (x, y) tuples in workspace coordinates
    """
    coordinates = []
    pairs = line.strip().split()
    
    for pair in pairs:
        if "," in pair:
            try:
                x_str, y_str = pair.split(",", 1)
                canvas_x = float(x_str.strip())  # 0-1400 range
                canvas_y = float(y_str.strip())  # 0-1000 range
                
                # Convert canvas coordinates to normalized (0-1) WITHOUT flipping
                # Let the FLIP_X and FLIP_Y flags in map_normalized_to_workspace handle orientation
                norm_x = canvas_x / 1400.0  # Direct mapping: 0→0, 1400→1
                norm_y = canvas_y / 1000.0  # Direct mapping: 0→0, 1000→1
                
                # Validate normalized coordinate ranges
                if not (0.0 <= norm_x <= 1.0 and 0.0 <= norm_y <= 1.0):
                    rospy.logwarn(f"Canvas coordinates out of range: Canvas({canvas_x}, {canvas_y}) -> Norm({norm_x:.3f}, {norm_y:.3f})")
                    continue
                
                # Map to workspace coordinates
                real_x, real_y = map_normalized_to_workspace(norm_x, norm_y)
                real_x, real_y = clamp_to_workspace_bounds(real_x, real_y)
                
                coordinates.append((real_x, real_y))
                
                # Debug logging for first few coordinates
                if len(coordinates) <= 3:
                    rospy.loginfo(f"Coord conversion: Canvas({canvas_x},{canvas_y}) → Norm({norm_x:.3f},{norm_y:.3f}) → FR3({real_x:.3f},{real_y:.3f})")
                
            except ValueError as e:
                rospy.logwarn(f"Error parsing coordinate pair '{pair}': {e}")
                continue
    
    return coordinates


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
    rospy.loginfo(f"Colors: 1/2/3 = select pencil colors, R = return current pencil")
    rospy.loginfo(f"Arrays: I = show status")
    rospy.loginfo(f"Current workspace: {SELECTED_PAPER} paper ({PAPER_WIDTH}x{PAPER_HEIGHT}m)")
    rospy.loginfo(f"ARRAY MODE: Processes coordinate arrays instead of real-time drawing")
    
    while not rospy.is_shutdown():
        c = getch()
        if c in ['1', '2', '3']:
            pencil_num = int(c)
            pickup_pencil_sequence(move_client, grasp_client, pencil_num)
        elif c in ['r', 'R'] and current_pencil is not None:
            # Return current pencil to its position
            return_pencil_to_position(move_client, grasp_client, current_pencil)
        elif c in ['+', '=']:
            # Adjust Z_ACTIVE with fine control
            Z_ACTIVE = min(MAX_Z_HEIGHT, Z_ACTIVE + Z_INCREMENT)
            rospy.loginfo(f"Z_ACTIVE height adjusted to: {Z_ACTIVE:.3f}")
        elif c in ['-', '_']:
            # Adjust Z_ACTIVE with fine control
            Z_ACTIVE = max(MIN_Z_HEIGHT, Z_ACTIVE - Z_INCREMENT)
            rospy.loginfo(f"Z_ACTIVE height adjusted to: {Z_ACTIVE:.3f}")
        elif c in ['i', 'I']:
            # Display system status
            current_pencil_name = PENCIL_POSITIONS[current_pencil]['name'] if current_pencil else "None"
            rospy.loginfo(f"=== SYSTEM STATUS ===")
            rospy.loginfo(f"Current pencil: {current_pencil_name}")
            rospy.loginfo(f"Pencil ready: {pencil_ready}")
            rospy.loginfo(f"Processing array: {processing_array}")
            rospy.loginfo(f"Arrays processed: {array_count}")
            rospy.loginfo(f"Z_ACTIVE: {Z_ACTIVE:.3f}m")
            rospy.loginfo(f"Z_LIFT: {Z_LIFT:.3f}m")
            rospy.loginfo(f"Last position: ({last_valid_x:.3f}, {last_valid_y:.3f})")
            rospy.loginfo(f"====================")
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
    """
    Receive coordinate arrays via TCP
    Each line contains an array of coordinates: "x1,y1 x2,y2 x3,y3 ..."
    """
    global processing_array

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        rospy.loginfo(f"Connecting to TCP server at {SERVER_IP}:{SERVER_PORT}...")
        sock.connect((SERVER_IP, SERVER_PORT))
        rospy.loginfo("Connected. Receiving coordinate arrays.")
        rospy.loginfo("Expected format: 'x1,y1 x2,y2 x3,y3 ...' (one array per line)")
        rospy.loginfo("VR coordinate system: (0,0)=bottom-right, (1000,1000)=top-left")
        
        # Add socket timeout to prevent indefinite blocking
        sock.settimeout(1.0)
        
        buffer = b""
        array_count_received = 0
        
        while not rospy.is_shutdown():
            try:
                data = sock.recv(4096)  # Increased buffer for arrays
                if not data:
                    rospy.logwarn("Connection closed by server.")
                    break

                buffer += data
                
                # Process all complete lines in buffer
                while b"\n" in buffer:
                    line, buffer = buffer.split(b"\n", 1)
                    array_count_received += 1
                    
                    try:
                        text = line.decode().strip()
                        if not text:  # Skip empty lines
                            continue
                        
                        rospy.loginfo(f"Received array #{array_count_received}: {len(text.split())} coordinate pairs")
                        
                        # Parse the coordinate array
                        coordinates = parse_coordinate_line(text)
                        
                        if len(coordinates) == 0:
                            rospy.logwarn(f"No valid coordinates found in array #{array_count_received}")
                            continue
                        
                        rospy.loginfo(f"Parsed {len(coordinates)} valid coordinates from array #{array_count_received}")
                        
                        # Wait for any current array processing to complete
                        while processing_array and not rospy.is_shutdown():
                            rospy.loginfo("Waiting for current array to complete...")
                            time.sleep(0.1)
                        
                        # Process the coordinate array
                        process_coordinate_array(coordinates)
                        
                    except Exception as e:
                        rospy.logwarn(f"Error processing array #{array_count_received}: {e}")
                        continue
                    
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

def main():
    global pub
    rospy.init_node("tcp_drawing_arrays")
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
    rospy.loginfo(f"=== ARRAY MODE CONFIGURATION ===")
    rospy.loginfo(f"Mode: Coordinate array processing")
    rospy.loginfo(f"Z_ACTIVE: {Z_ACTIVE:.3f}m")
    rospy.loginfo(f"Z_LIFT: {Z_LIFT:.3f}m")
    rospy.loginfo(f"===============================")

    threading.Thread(target=keyboard_listener, daemon=True).start()
    threading.Thread(target=tcp_receiver, daemon=True).start()

    # Keep the node alive
    rospy.spin()

if __name__ == "__main__":
    main()