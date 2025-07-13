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
from franka_msgs.msg import FrankaState

# === TCP Configuration ===
SERVER_IP = "127.0.0.1" # 127.0.0.1
SERVER_PORT = 5005

# === Workspace Configuration ===
PAPER_SIZES = {
    "A2": (0.420, 0.594),
    "A3": (0.253, 0.382),
    "A4": (0.210, 0.297),
    "A5": (0.148, 0.210)
}

SELECTED_PAPER = "A3"
PAPER_WIDTH, PAPER_HEIGHT = PAPER_SIZES[SELECTED_PAPER]

WORKSPACE_CENTER_X = 0.48
WORKSPACE_CENTER_Y = 0.0

# Coordinate transformation flags
FLIP_X = True
FLIP_Y = True

# Workspace boundaries
WORKSPACE_X_MIN = WORKSPACE_CENTER_X - PAPER_WIDTH / 2
WORKSPACE_X_MAX = WORKSPACE_CENTER_X + PAPER_WIDTH / 2
WORKSPACE_Y_MIN = WORKSPACE_CENTER_Y - PAPER_HEIGHT / 2
WORKSPACE_Y_MAX = WORKSPACE_CENTER_Y + PAPER_HEIGHT / 2

# === Height Configuration ===
MIN_Z_HEIGHT = 0.05
MAX_Z_HEIGHT = 0.25

# Base heights for thick and tight pencils
Z_ACTIVE_THICK_BASE = 0.068   # Base height for thick pencils (ends with 'b'), it was 115
Z_ACTIVE_TIGHT_BASE = 0.127   # Base height for tight pencils (ends with 's')

Z_IDLE = 0.1
Z_LIFT = 0.200
Z_INCREMENT = 0.001
HEIGHT_TOLERANCE = 0.001  # 1mm tolerance for feedback

# Height adjustment offsets (applied to both thick and tight)
thick_offset = 0.0  # Offset for thick pencil heights
tight_offset = 0.0  # Offset for tight pencil heights

# Current Z_ACTIVE (will be set based on pencil type)
Z_ACTIVE = Z_ACTIVE_THICK_BASE  # Default to thick

# Fixed orientation for end effector
FIXED_ORIENTATION = {"x": 1.0, "y": 0.0, "z": 0.0, "w": 0.0}

# === Pencil System ===
PENCIL_BASE_X = 0.35 # 0.326
PENCIL_BASE_Y_LEFT = -0.29
PENCIL_BASE_Y_RIGHT = 0.29
PENCIL_BASE_Z = 0.066  # Fixed pickup height 115 good
PENCIL_X_OFFSET = 0.06

# Pencil positions (20 colors)
PENCIL_POSITIONS = {}
for i in range(1, 21):
    if i <= 5:  # Left side, thick
        pos = {"x": PENCIL_BASE_X + (i-1)*PENCIL_X_OFFSET, "y": PENCIL_BASE_Y_LEFT, "z": PENCIL_BASE_Z}
    if i >= 6 and i <= 10:  # Right side, thick
        pos = {"x": PENCIL_BASE_X + (i-6)*PENCIL_X_OFFSET, "y": PENCIL_BASE_Y_RIGHT, "z": PENCIL_BASE_Z}
    if i >= 11 and i <= 15:  # Left side, tight
        pos = {"x": PENCIL_BASE_X + (i-11)*PENCIL_X_OFFSET, "y": PENCIL_BASE_Y_LEFT - 0.107, "z": 0.132}
    if i >= 16: # Right side, tight
        pos = {"x": PENCIL_BASE_X + (i-16)*PENCIL_X_OFFSET, "y": PENCIL_BASE_Y_RIGHT + 0.107, "z": 0.132}
    PENCIL_POSITIONS[i] = pos

# Color code to pencil mapping
COLOR_CODE_TO_PENCIL = {
    # Thick colors ("b" at the end)
    "#16160Fb": 10,  # Black
    "#000000b": 10,  # Black alternative
    "#7A3D28b": 2,  # Brown
    "#D82929b": 3,  # Red
    "#E86E09b": 4,  # Orange
    "#DBC416b": 5,  # Yellow
    "#49C893b": 6,  # Light Green
    "#02704Db": 7,  # Dark Green
    "#0D2875b": 8,  # Dark Blue
    "#0295D5b": 9,  # Light Blue
    "#6B6B6Bb": 1,  # Grey

    # Tight colors
    "#16160Fs": 11,  # Black
    "#000000s": 11,  # Black alternative
    "#7A3D28s": 12,  # Brown
    "#D82929s": 13,  # Red
    "#E86E09s": 14,  # Orange
    "#DBC416s": 15,  # Yellow
    "#49C893s": 16,  # Light Green
    "#02704Ds": 17,  # Dark Green
    "#0D2875s": 18,  # Dark Blue
    "#0295D5s": 19,  # Light Blue
    "#C0BDAEs": 20,  # White
}


# === Global State ===
current_pencil = None
current_color_code = None
current_pencil_type = None  # 'thick' or 'tight'
processing_array = False
array_count = 0
last_valid_x = WORKSPACE_CENTER_X
last_valid_y = WORKSPACE_CENTER_Y
current_z_height = Z_IDLE
pub = None
current_ee_z = 0.0  # Current end-effector Z position from feedback

# Gripper clients
move_client = None
grasp_client = None
gripper_available = False

def update_z_active():
    """Update Z_ACTIVE based on current pencil type and offsets"""
    global Z_ACTIVE
    
    if current_pencil_type == 'thick':
        Z_ACTIVE = Z_ACTIVE_THICK_BASE + thick_offset
    elif current_pencil_type == 'tight':
        Z_ACTIVE = Z_ACTIVE_TIGHT_BASE + tight_offset
    else:
        Z_ACTIVE = Z_ACTIVE_THICK_BASE + thick_offset  # Default to thick
    
    # Ensure Z_ACTIVE stays within bounds
    Z_ACTIVE = max(MIN_Z_HEIGHT, min(Z_ACTIVE, MAX_Z_HEIGHT))

def get_pencil_type(color_code):
    """Determine if a color code is for thick or tight pencil"""
    if color_code and color_code.endswith('b'):
        return 'thick'
    elif color_code and color_code.endswith('s'):
        return 'tight'
    return None

def franka_state_callback(msg):
    """Callback to get current end-effector position from robot state"""
    global current_ee_z
    # O_T_EE is a 4x4 transformation matrix flattened to 16 elements
    # Element 14 (index 14) contains the Z position
    if len(msg.O_T_EE) > 14:
        current_ee_z = msg.O_T_EE[14]

def getch():
    """Get single character input without pressing Enter"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def clamp_to_workspace(x, y):
    """Clamp coordinates to workspace boundaries"""
    x_clamped = max(WORKSPACE_X_MIN, min(x, WORKSPACE_X_MAX))
    y_clamped = max(WORKSPACE_Y_MIN, min(y, WORKSPACE_Y_MAX))
    return x_clamped, y_clamped

def map_normalized_to_workspace(norm_x, norm_y):
    """Map normalized canvas coordinates (0-1) to FR3 workspace coordinates"""
    # Apply coordinate flips
    if FLIP_X:
        norm_x = 1.0 - norm_x
    if FLIP_Y:
        norm_y = 1.0 - norm_y
    
    # Swap X and Y mapping: Canvas X→FR3 Y, Canvas Y→FR3 X
    real_x = WORKSPACE_X_MIN + norm_y * PAPER_WIDTH
    real_y = WORKSPACE_Y_MIN + norm_x * PAPER_HEIGHT
    
    return real_x, real_y

def publish_pose(x, y, z):
    """Publish pose to robot"""
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

def wait_for_height(target_x, target_y, target_z, timeout=10.0):
    """
    Publish pose and wait until end-effector reaches target height within tolerance
    Returns True if reached, False if timeout
    """
    global current_ee_z
    
    start_time = time.time()
    rate = rospy.Rate(50)  # 50Hz publishing rate
    
    rospy.loginfo(f"Waiting for height {target_z:.3f}m (current: {current_ee_z:.3f}m)")
    
    while not rospy.is_shutdown():
        # Check if we've reached the target height
        height_error = abs(current_ee_z - target_z)
        
        if height_error <= HEIGHT_TOLERANCE:
            rospy.loginfo(f"Height reached: {current_ee_z:.3f}m (error: {height_error*1000:.1f}mm)")
            return True
        
        # Check timeout
        if time.time() - start_time > timeout:
            rospy.logwarn(f"Height timeout: target {target_z:.3f}m, current {current_ee_z:.3f}m")
            return False
        
        # Continue publishing the target pose
        publish_pose(target_x, target_y, target_z)
        rate.sleep()
    
    return False

def gradual_movement(start_x, start_y, start_z, end_x, end_y, end_z, steps=5, delay=0.3):
    """Perform gradual movement between two points"""
    for step in range(1, steps + 1):
        progress = step / steps
        x = start_x + (end_x - start_x) * progress
        y = start_y + (end_y - start_y) * progress
        z = start_z + (end_z - start_z) * progress
        publish_pose(x, y, z)
        time.sleep(delay)

def pickup_pencil(pencil_number):
    """Pick up specified pencil using feedback-based positioning"""
    global current_pencil, last_valid_x, last_valid_y, current_z_height

    #print("PORCODIO CANE")
    
    if pencil_number not in PENCIL_POSITIONS:
        rospy.logwarn(f"Invalid pencil number: {pencil_number}")
        return False
    
    pencil_pos = PENCIL_POSITIONS[pencil_number]
    approach_z = pencil_pos['z'] + 0.3  # 20cm above pickup position
    
    rospy.loginfo(f"Picking up pencil {pencil_number}")
    
    # Return current pencil if holding one
    if current_pencil is not None and current_pencil != pencil_number:
        #print("PORCODIO BASTARDO")
        return_pencil(current_pencil)
    
    # If already holding the requested pencil, return
    if current_pencil == pencil_number:
        rospy.loginfo(f"Already holding pencil {pencil_number}")
        return True
    #print("STILL HERE")
    
    # Open gripper
    if gripper_available:
        move_goal = MoveGoal(width=0.05, speed=0.1)
        move_client.send_goal(move_goal)
        move_client.wait_for_result()
    
    # Move to approach position using gradual movement for XY, then feedback for Z
    gradual_movement(last_valid_x, last_valid_y, current_z_height,
                    pencil_pos['x'], pencil_pos['y'], approach_z, steps=6, delay=0.4)

    # Move in the middle
    gradual_movement(pencil_pos['x'], pencil_pos['y'], approach_z,
                    pencil_pos['x'], pencil_pos['y'], pencil_pos['z'], steps=6, delay=0.4)
    
    # Descend to pickup position using feedback
    if not wait_for_height(pencil_pos['x'], pencil_pos['y'], pencil_pos['z'], timeout=10.0):
        rospy.logerr("Failed to reach pickup height")
        return False

    # Close gripper for 30mm pencil
    if gripper_available:
        grasp_goal = GraspGoal(width=0.032, speed=0.05, force=17.0)  # 32mm width for 30mm pencil
        grasp_goal.epsilon.inner = 0.001
        grasp_goal.epsilon.outer = 0.005
        grasp_client.send_goal(grasp_goal)
        grasp_client.wait_for_result()
    
    time.sleep(0.5)
    
    # Lift pencil using feedback (gradual)
    gradual_movement(pencil_pos['x'], pencil_pos['y'], pencil_pos['z'],
                    pencil_pos['x'], pencil_pos['y'], approach_z, steps=6, delay=0.4)
    
    # Move to drawing area
    gradual_movement(pencil_pos['x'], pencil_pos['y'], approach_z,
                    WORKSPACE_CENTER_X, WORKSPACE_CENTER_Y, Z_IDLE, steps=6, delay=0.4)
    
    # Update state
    current_pencil = pencil_number
    last_valid_x = WORKSPACE_CENTER_X
    last_valid_y = WORKSPACE_CENTER_Y
    current_z_height = Z_IDLE
    
    rospy.loginfo(f"Pencil {pencil_number} pickup complete")
    return True

def return_pencil(pencil_number):
    """Return pencil using feedback-based positioning"""
    global current_pencil, last_valid_x, last_valid_y, current_z_height
    
    if pencil_number not in PENCIL_POSITIONS:
        return False
    
    pencil_pos = PENCIL_POSITIONS[pencil_number]
    approach_z = pencil_pos['z'] + 0.3
    
    rospy.loginfo(f"Returning pencil {pencil_number}")
    
    # Move to approach position using gradual movement for XY, then feedback for Z
    gradual_movement(last_valid_x, last_valid_y, current_z_height,
                    pencil_pos['x'], pencil_pos['y'], approach_z, steps=6, delay=0.4)

    # Move in the middle
    gradual_movement(pencil_pos['x'], pencil_pos['y'], approach_z,
                    pencil_pos['x'], pencil_pos['y'], pencil_pos['z']+0.04, steps=6, delay=0.4)
    
    
    # Descend to pencil base height using feedback
    if not wait_for_height(pencil_pos['x'], pencil_pos['y'], pencil_pos['z']+0.04, timeout=10.0):
        rospy.logerr("Failed to reach drop height")
        return False
    
    # Wait a moment to ensure robot is stable at the correct height
    time.sleep(0.5)
    
    # Open gripper to release pencil
    if gripper_available:
        move_goal = MoveGoal(width=0.05, speed=0.1)
        move_client.send_goal(move_goal)
        move_client.wait_for_result()
    
    # Give time for pencil to drop completely
    time.sleep(0.5)
    
    # Lift back to approach position using feedback
    if not wait_for_height(pencil_pos['x'], pencil_pos['y'], approach_z, timeout=10.0):
        rospy.logerr("Failed to lift after dropping pencil")
        return False
    
    # Update state - Robot is now at approach height above the pencil
    current_pencil = None
    last_valid_x = pencil_pos['x']
    last_valid_y = pencil_pos['y']
    current_z_height = approach_z
    
    rospy.loginfo(f"Pencil {pencil_number} returned at height {PENCIL_BASE_Z:.3f}m")
    return True

def process_coordinate_array(coordinates, color_changed=False):
    """Process array of coordinates for drawing"""
    global last_valid_x, last_valid_y, current_z_height, processing_array, array_count
    
    if len(coordinates) == 0:
        return
    
    processing_array = True
    array_count += 1
    
    rospy.loginfo(f"Processing array #{array_count} with {len(coordinates)} coordinates")
    
    # Lift end effector
    publish_pose(last_valid_x, last_valid_y, Z_LIFT)
    time.sleep(0.3)
    
    # Move to first coordinate at lift height
    first_x, first_y = coordinates[0]
    publish_pose(first_x, first_y, Z_LIFT)
    time.sleep(0.3)
    
    # Gradual descent to drawing height (use current Z_ACTIVE)
    gradual_movement(first_x, first_y, Z_LIFT,
                    first_x, first_y, Z_ACTIVE, steps=5, delay=0.2)
    
    # Execute drawing path
    for i, (x, y) in enumerate(coordinates):
        publish_pose(x, y, Z_ACTIVE)
        last_valid_x = x
        last_valid_y = y
        time.sleep(0.05)  # Small delay for smooth movement
    
    # Lift after completing array
    publish_pose(last_valid_x, last_valid_y, Z_LIFT)
    time.sleep(0.3)
    current_z_height = Z_LIFT
    
    processing_array = False
    rospy.loginfo(f"Array #{array_count} completed")

def parse_coordinate_line(line):
    """Parse coordinate line into workspace coordinates"""
    coordinates = []
    pairs = line.strip().split()
    
    for pair in pairs:
        if "," in pair:
            try:
                x_str, y_str = pair.split(",", 1)
                canvas_x = float(x_str.strip())  # 0-1400 range
                canvas_y = float(y_str.strip())  # 0-1000 range
                
                # Normalize to 0-1 range
                norm_x = canvas_x / 1400.0
                norm_y = canvas_y / 1000.0
                
                # Validate range
                if not (0.0 <= norm_x <= 1.0 and 0.0 <= norm_y <= 1.0):
                    continue
                
                # Map to workspace
                real_x, real_y = map_normalized_to_workspace(norm_x, norm_y)
                real_x, real_y = clamp_to_workspace(real_x, real_y)
                
                coordinates.append((real_x, real_y))
                
            except ValueError:
                continue
    
    return coordinates

def height_adjustment_listener():
    """Listen for +/- keys to adjust Z_ACTIVE height for current pencil type"""
    global thick_offset, tight_offset
    
    rospy.loginfo("Height adjustment ready: +/- keys to adjust drawing height for current pencil type")
    
    while not rospy.is_shutdown():
        try:
            c = getch()
            if c in ['+', '=']:
                if current_pencil_type == 'thick':
                    thick_offset = min(MAX_Z_HEIGHT - Z_ACTIVE_THICK_BASE, thick_offset + Z_INCREMENT)
                    update_z_active()
                    rospy.loginfo(f"Thick pencil Z_ACTIVE: {Z_ACTIVE:.3f}m (offset: +{thick_offset:.3f}m)")
                elif current_pencil_type == 'tight':
                    tight_offset = min(MAX_Z_HEIGHT - Z_ACTIVE_TIGHT_BASE, tight_offset + Z_INCREMENT)
                    update_z_active()
                    rospy.loginfo(f"Tight pencil Z_ACTIVE: {Z_ACTIVE:.3f}m (offset: +{tight_offset:.3f}m)")
                else:
                    # No pencil selected, adjust thick as default
                    thick_offset = min(MAX_Z_HEIGHT - Z_ACTIVE_THICK_BASE, thick_offset + Z_INCREMENT)
                    update_z_active()
                    rospy.loginfo(f"Default (thick) Z_ACTIVE: {Z_ACTIVE:.3f}m (offset: +{thick_offset:.3f}m)")
                    
            elif c in ['-', '_']:
                if current_pencil_type == 'thick':
                    thick_offset = max(MIN_Z_HEIGHT - Z_ACTIVE_THICK_BASE, thick_offset - Z_INCREMENT)
                    update_z_active()
                    rospy.loginfo(f"Thick pencil Z_ACTIVE: {Z_ACTIVE:.3f}m (offset: {thick_offset:+.3f}m)")
                elif current_pencil_type == 'tight':
                    tight_offset = max(MIN_Z_HEIGHT - Z_ACTIVE_TIGHT_BASE, tight_offset - Z_INCREMENT)
                    update_z_active()
                    rospy.loginfo(f"Tight pencil Z_ACTIVE: {Z_ACTIVE:.3f}m (offset: {tight_offset:+.3f}m)")
                else:
                    # No pencil selected, adjust thick as default
                    thick_offset = max(MIN_Z_HEIGHT - Z_ACTIVE_THICK_BASE, thick_offset - Z_INCREMENT)
                    update_z_active()
                    rospy.loginfo(f"Default (thick) Z_ACTIVE: {Z_ACTIVE:.3f}m (offset: {thick_offset:+.3f}m)")
                    
            elif c in ['q', 'Q']:
                # Return pencil before quitting
                if current_pencil is not None:
                    return_pencil(current_pencil)
                rospy.signal_shutdown("User quit")
                break
        except:
            break


def tcp_receiver():
    """Receive and process coordinate arrays via TCP"""
    global processing_array, current_color_code, current_pencil_type
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        rospy.loginfo(f"Connecting to TCP server at {SERVER_IP}:{SERVER_PORT}")
        sock.connect((SERVER_IP, SERVER_PORT))
        rospy.loginfo("Connected. Waiting for coordinate arrays...")
        
        sock.settimeout(1.0)
        buffer = b""
        
        while not rospy.is_shutdown():
            try:
                data = sock.recv(4096)
                if not data:
                    break
                
                buffer += data
                
                # Process complete lines
                while b"\n" in buffer:
                    line, buffer = buffer.split(b"\n", 1)
                    
                    try:
                        text = line.decode().strip()
                        if not text:
                            continue
                        
                        # Debug: Log the received line
                        rospy.loginfo(f"Received line: {text}")
                        
                        # Extract color code - look for patterns like #xxxxxxb or #xxxxxxs
                        color_code = None
                        coords_text = text
                        
                        parts = text.split()
                        if len(parts) > 0 and parts[0].startswith('#'):
                            potential_color = parts[0]  # Keep original case
                            
                            # Check if it's 8 characters and ends with 'b' or 's' (lowercase)
                            if (len(potential_color) == 8 and 
                                potential_color.endswith(('b', 's'))):
                                color_code = potential_color
                                coords_text = ' '.join(parts[1:])
                                rospy.loginfo(f"Extracted color code: {color_code}")
                            else:
                                rospy.logwarn(f"Invalid color code format: {potential_color} (length: {len(potential_color)})")
                        
                        # Parse coordinates
                        coordinates = parse_coordinate_line(coords_text)
                        if len(coordinates) == 0:
                            rospy.logwarn("No valid coordinates found in line")
                            continue
                        
                        rospy.loginfo(f"Parsed {len(coordinates)} coordinates")
                        
                        # Handle pencil selection for color changes
                        color_changed = False
                        if color_code:
                            if color_code in COLOR_CODE_TO_PENCIL:
                                pencil_number = COLOR_CODE_TO_PENCIL[color_code]
                                new_pencil_type = get_pencil_type(color_code)
                                
                                if color_code != current_color_code:
                                    color_changed = True
                                    rospy.loginfo(f"Color change from {current_color_code} to {color_code}, selecting pencil {pencil_number}")
                                    
                                    # Wait for current array to complete
                                    while processing_array and not rospy.is_shutdown():
                                        time.sleep(0.1)
                                    
                                    rospy.loginfo(f"Picking up pencil {pencil_number} for color {color_code}")
                                    if pickup_pencil(pencil_number):
                                        current_color_code = color_code
                                        current_pencil_type = new_pencil_type
                                        update_z_active()  # Update Z_ACTIVE based on pencil type
                                        rospy.loginfo(f"Successfully picked up {new_pencil_type} pencil {pencil_number}, Z_ACTIVE: {Z_ACTIVE:.3f}m")
                                    else:
                                        rospy.logerr(f"Failed to pick up pencil {pencil_number}")
                                else:
                                    rospy.loginfo(f"Already using color {color_code} (pencil {COLOR_CODE_TO_PENCIL[color_code]})")
                            else:
                                rospy.logwarn(f"Color code {color_code} not found in mapping. Available codes: {list(COLOR_CODE_TO_PENCIL.keys())}")
                        else:
                            rospy.loginfo("No color code found, using current pencil")
                        
                        # Wait for processing to complete before new array
                        while processing_array and not rospy.is_shutdown():
                            time.sleep(0.1)
                        
                        # Process the coordinate array
                        process_coordinate_array(coordinates, color_changed)
                        
                    except Exception as e:
                        rospy.logwarn(f"Error processing line '{text}': {e}")
                        import traceback
                        rospy.logwarn(f"Traceback: {traceback.format_exc()}")
                        continue
                        
            except socket.timeout:
                continue
            except Exception as e:
                rospy.logerr(f"TCP receive error: {e}")
                break
                
    except Exception as e:
        rospy.logerr(f"TCP connection failed: {e}")
    finally:
        sock.close()


def initialize_gripper():
    """Initialize gripper action clients"""
    global move_client, grasp_client, gripper_available
    
    move_client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
    grasp_client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)
    
    try:
        rospy.loginfo("Connecting to gripper servers...")
        if (move_client.wait_for_server(rospy.Duration(2.0)) and 
            grasp_client.wait_for_server(rospy.Duration(2.0))):
            gripper_available = True
            rospy.loginfo("Gripper servers connected")
        else:
            rospy.logwarn("Gripper servers unavailable")
    except Exception as e:
        rospy.logwarn(f"Gripper connection failed: {e}")


def print_color_mapping():
    """Print the color code to pencil mapping for debugging"""
    rospy.loginfo("=== COLOR CODE MAPPING ===")
    for color_code, pencil_num in COLOR_CODE_TO_PENCIL.items():
        pencil_type = "thick" if color_code.endswith('b') else "tight"
        rospy.loginfo(f"  {color_code} -> Pencil {pencil_num} ({pencil_type})")
    rospy.loginfo("===========================")



def main():
    global pub
    
    rospy.init_node("simplified_tcp_drawing")
    pub = rospy.Publisher("/demo/pose_final", PoseStamped, queue_size=10)
    
    # Subscribe to robot state for end-effector feedback
    rospy.Subscriber("/franka_state_controller/franka_states", FrankaState, franka_state_callback)

    # Print color mapping for debugging
    print_color_mapping()
    
    # Log configuration
    rospy.loginfo(f"=== SIMPLIFIED TCP DRAWING BRIDGE ===")
    rospy.loginfo(f"Paper: {SELECTED_PAPER} ({PAPER_WIDTH}x{PAPER_HEIGHT}m)")
    rospy.loginfo(f"Workspace center: ({WORKSPACE_CENTER_X}, {WORKSPACE_CENTER_Y})")
    rospy.loginfo(f"Thick pencil Z_ACTIVE base: {Z_ACTIVE_THICK_BASE:.3f}m")
    rospy.loginfo(f"Tight pencil Z_ACTIVE base: {Z_ACTIVE_TIGHT_BASE:.3f}m")
    rospy.loginfo(f"Current Z_ACTIVE: {Z_ACTIVE:.3f}m")
    rospy.loginfo(f"Pencil pickup height: {PENCIL_BASE_Z:.3f}m")
    rospy.loginfo(f"Height tolerance: {HEIGHT_TOLERANCE*1000:.1f}mm")
    rospy.loginfo(f"Controls: +/- adjust height for current pencil type, Q quit")
    rospy.loginfo(f"======================================")
    
    # Initialize gripper
    initialize_gripper()
    
    # Start threads
    threading.Thread(target=height_adjustment_listener, daemon=True).start()
    threading.Thread(target=tcp_receiver, daemon=True).start()
    
    # Keep node alive
    rospy.spin()

if __name__ == "__main__":
    main()