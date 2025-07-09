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

# === TCP Parameters ===
SERVER_IP = "127.0.0.1" # 127.0.0.1   192.168.1.108
SERVER_PORT = 5005

# === WORKSPACE CONFIGURATION ===
# Paper sizes in meters (width x height)
PAPER_SIZES = {
    "A2": (0.420, 0.594),
    "A3": (0.233, 0.362), # Reduced of 2cm
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
FLIP_X = True   # Set to True because right GUI -> left robot (needs flip)
FLIP_Y = True  # Y seems correct based on description

# Calculate workspace boundaries based on paper size and center
WORKSPACE_X_MIN = WORKSPACE_CENTER_X - PAPER_WIDTH / 2
WORKSPACE_X_MAX = WORKSPACE_CENTER_X + PAPER_WIDTH / 2
WORKSPACE_Y_MIN = WORKSPACE_CENTER_Y - PAPER_HEIGHT / 2
WORKSPACE_Y_MAX = WORKSPACE_CENTER_Y + PAPER_HEIGHT / 2

# === Height and Orientation ===
MIN_Z_HEIGHT = 0.09
MAX_Z_HEIGHT = 0.25
Z_ACTIVE_BIG = 0.115
Z_ACTIVE_SMALL = 0.127 # They start drawing at 0.128
Z_ACTIVE = Z_ACTIVE_SMALL  # Default (can be overwritten when pencil is picked)
Z_IDLE = 0.150 # Seems good
CURRENT_Z_HEIGHT = Z_IDLE
Z_INCREMENT = 0.001  # Fine increment for Z_ACTIVE adjustments

# Z-offset for safe approach from above (in meters)
Z_APPROACH_OFFSET = 0.15

FIXED_ORIENTATION = {
    "x": 1.0,
    "y": 0.0,
    "z": 0.0,
    "w": 0.0
}

# === Gripper State ===
gripper_closed = False
gripper_available = False

# === PENCIL/COLOR SYSTEM ===
# Base pencil pickup position
PENCIL_BASE_X = 0.4        # Starting X coordinate for pencil pickup
PENCIL_BASE_Y_LEFT = -0.30  # Y coordinate for left pencils
PENCIL_BASE_Y_RIGHT = 0.30  # Y coordinate for right pencils
PENCIL_BASE_Z = 0.115        # Z coordinate for pencil pickup
PENCIL_X_OFFSET = 0.06      # 8cm offset along X between pencils

# Pencil positions for each color
PENCIL_POSITIONS = {}
for i in range(1, 21):
    pos = {}
    if i <= 5:  # Left side, thick
        pos = {"x": PENCIL_BASE_X + (i-1)*PENCIL_X_OFFSET, "y": PENCIL_BASE_Y_LEFT, "z": PENCIL_BASE_Z}
    elif i >= 6 and i <= 10:  # Right side, thick
        pos = {"x": PENCIL_BASE_X + (i-6)*PENCIL_X_OFFSET, "y": PENCIL_BASE_Y_RIGHT, "z": PENCIL_BASE_Z}
    elif i >= 11 and i <= 15:  # Left side, tight
        pos = {"x": PENCIL_BASE_X + (i-11)*PENCIL_X_OFFSET, "y": PENCIL_BASE_Y_LEFT - 0.107, "z": 0.132}
    elif i >= 16: # Right side, tight
        pos = {"x": PENCIL_BASE_X + (i-16)*PENCIL_X_OFFSET, "y": PENCIL_BASE_Y_RIGHT + 0.107, "z": 0.132}
    
    # FIX: Replaced f-string with .format() for compatibility
    pos["name"] = "Pencil {}".format(i)
    PENCIL_POSITIONS[i] = pos


# Color code to pencil mapping
COLOR_CODE_TO_PENCIL = {
    # Thick colors ("b" at the end)
    "#16160Fb": 1,  # Black
    "#000000b": 1,  # Black alternative
    "#7A3D28b": 2,  # Brown
    "#D82929b": 3,  # Red
    "#E86E09b": 4,  # Orange
    "#DBC416b": 5,  # Yellow
    "#49C893b": 6,  # Light Green
    "#02704Db": 7,  # Dark Green
    "#0D2875b": 8,  # Dark Blue
    "#0295D5b": 9,  # Light Blue
    "#C0BDAEb": 10,  # White

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

last_pencil = None
current_pencil = None
pencil_ready = False
first_coordinate_after_idle = True
gradual_approach_in_progress = False

# === Robot State ===
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
    """
    if FLIP_X:
        norm_x = 1.0 - norm_x
    if FLIP_Y:
        norm_y = 1.0 - norm_y
    
    real_x = WORKSPACE_X_MIN + norm_y * PAPER_WIDTH
    real_y = WORKSPACE_Y_MIN + norm_x * PAPER_HEIGHT
    
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
    rospy.loginfo("Starting gradual movement: ({:.3f}, {:.3f}, {:.3f}) -> ({:.3f}, {:.3f}, {:.3f})".format(start_x, start_y, start_z, end_x, end_y, end_z))
    
    for step in range(1, steps + 1):
        progress = step / steps
        x = start_x + (end_x - start_x) * progress
        y = start_y + (end_y - start_y) * progress
        z = start_z + (end_z - start_z) * progress
        
        publish_pose(x, y, z)
        time.sleep(delay)

def return_pencil_to_position(move_client, grasp_client, pencil_number):
    """
    Return the current pencil to its designated position
    """
    global sock, gripper_closed, current_pencil, pencil_ready, CURRENT_Z_HEIGHT, last_valid_x, last_valid_y, last_pencil
    
    if pencil_number not in PENCIL_POSITIONS:
        rospy.logwarn("Invalid pencil number: {}".format(pencil_number))
        return

    if sock:
        sock.sendall("STOP\n".encode())
    
    pencil_pos = PENCIL_POSITIONS[pencil_number]
    approach_z = pencil_pos['z'] + Z_APPROACH_OFFSET
    
    rospy.loginfo("Returning {} to its holder...".format(pencil_pos['name']))
    
    perform_gradual_movement(
        last_valid_x, last_valid_y, CURRENT_Z_HEIGHT,
        pencil_pos['x'], pencil_pos['y'], approach_z,
        steps=4, delay=0.6
    )
    
    perform_gradual_movement(
        pencil_pos['x'], pencil_pos['y'], approach_z,
        pencil_pos['x'], pencil_pos['y'], pencil_pos['z'] + 0.03,
        steps=4, delay=0.5
    )
    
    rospy.loginfo("Releasing pencil...")
    if gripper_available:
        move_goal = MoveGoal(width=0.05, speed=0.1)
        move_client.send_goal(move_goal)
        move_client.wait_for_result()
    gripper_closed = False
    time.sleep(1.0)
    
    perform_gradual_movement(
        pencil_pos['x'], pencil_pos['y'], pencil_pos['z'],
        pencil_pos['x'], pencil_pos['y'], approach_z,
        steps=4, delay=0.4
    )

    CURRENT_Z_HEIGHT = approach_z
    current_pencil = None
    last_pencil = pencil_number
    pencil_ready = False
    
    rospy.loginfo("{} returned successfully.".format(pencil_pos['name']))

def pickup_pencil_sequence(move_client, grasp_client, pencil_number):
    """
    Pick up a specific pencil/color
    """
    global sock, gripper_closed, current_pencil, pencil_ready, last_valid_x, last_valid_y, CURRENT_Z_HEIGHT, last_pencil
    
    if pencil_number not in PENCIL_POSITIONS:
        rospy.logwarn("Invalid pencil number: {}".format(pencil_number))
        return
    
    pencil_pos = PENCIL_POSITIONS[pencil_number]
    approach_z = pencil_pos['z'] + Z_APPROACH_OFFSET
    
    if current_pencil is not None and current_pencil != pencil_number:
        rospy.loginfo("Returning current pencil ({}) before picking up {}".format(PENCIL_POSITIONS[current_pencil]['name'], pencil_pos['name']))
        return_pencil_to_position(move_client, grasp_client, current_pencil)
    
    if current_pencil == pencil_number and pencil_ready:
        rospy.loginfo("{} is already selected and ready.".format(pencil_pos['name']))
        return
    
    rospy.loginfo("Starting pickup sequence for {}...".format(pencil_pos['name']))
    
    rospy.loginfo("Opening gripper...")
    if gripper_available:
        move_goal = MoveGoal(width=0.05, speed=0.1)
        move_client.send_goal(move_goal)
        move_client.wait_for_result()
    gripper_closed = False
    time.sleep(1.0)
    
    # Determine if we need gradual movement based on side switching
    need_gradual_movement = False
    if last_pencil is not None:
        # Determine sides based on pencil numbers
        current_side = get_pencil_side(last_pencil)
        target_side = get_pencil_side(pencil_number)
        
        if current_side != target_side:
            need_gradual_movement = True
            rospy.loginfo("Switching from {} side to {} side - using gradual movement".format(current_side, target_side))
        else:
            rospy.loginfo("Staying on {} side - using simple movement".format(current_side))
    else:
        rospy.loginfo("No current pencil - using simple movement")
    
    # Move to pencil approach position
    if need_gradual_movement:
        # Use gradual movement when switching sides
        rospy.loginfo("Using gradual movement to approach pencil position")
        perform_gradual_movement(
            #WORKSPACE_CENTER_X, WORKSPACE_CENTER_Y, approach_z,
            PENCIL_POSITIONS[last_pencil]['x'], PENCIL_POSITIONS[last_pencil]['y'], approach_z,
            pencil_pos['x'], pencil_pos['y'], approach_z,
            steps=4, delay=0.6
        )
    else:
        # Use simple movement when staying on same side or first pickup
        rospy.loginfo("Using simple movement to approach pencil position")
        publish_pose(pencil_pos['x'], pencil_pos['y'], approach_z)
        time.sleep(1.0)  # Wait for movement to complete
    
    # Descend to pencil position
    perform_gradual_movement(
        pencil_pos['x'], pencil_pos['y'], approach_z,
        pencil_pos['x'], pencil_pos['y'], pencil_pos['z'],
        steps=6, delay=0.5
    )
    
    time.sleep(1.0)
    
    rospy.loginfo("Grasping {}...".format(pencil_pos['name']))
    
    if gripper_available:
        grasp_goal = GraspGoal(width=0.032, speed=0.05, force=20.0)  # 32mm width for 30mm pencil
        grasp_goal.epsilon.inner = 0.001
        grasp_goal.epsilon.outer = 0.005
        grasp_client.send_goal(grasp_goal)
        grasp_client.wait_for_result()
    time.sleep(1.0)
    
    # Lift pencil
    perform_gradual_movement(
        pencil_pos['x'], pencil_pos['y'], pencil_pos['z'],
        pencil_pos['x'], pencil_pos['y'], approach_z,
        steps=4, delay=0.6
    )
    
    # Move to drawing area
    perform_gradual_movement(
        pencil_pos['x'], pencil_pos['y'], approach_z,
        WORKSPACE_CENTER_X, WORKSPACE_CENTER_Y, Z_IDLE,
        steps=6, delay=0.6
    )
    
    if sock:
        sock.sendall("GO\n".encode())

    CURRENT_Z_HEIGHT = Z_IDLE
    current_pencil = pencil_number
    pencil_ready = True

    # Adjust Z_ACTIVE based on pencil size
    global Z_ACTIVE
    if pencil_number <= 10:  # Big pencil (thick grip)
        Z_ACTIVE = Z_ACTIVE_BIG
    else:  # Small pencil (tight grip)
        Z_ACTIVE = Z_ACTIVE_SMALL
    rospy.loginfo("Z_ACTIVE set to {:.3f} based on pencil size.".format(Z_ACTIVE))

    # Update last valid position to drawing area center
    last_valid_x = WORKSPACE_CENTER_X
    last_valid_y = WORKSPACE_CENTER_Y

    rospy.loginfo("{} pickup complete. Robot is ready for drawing.".format(pencil_pos['name']))

def get_pencil_side(pencil_number):
    """
    Determine which side a pencil is on based on its number
    """
    if pencil_number <= 5:  # Left side, thick
        return "left"
    elif pencil_number >= 6 and pencil_number <= 10:  # Right side, thick
        return "right"
    elif pencil_number >= 11 and pencil_number <= 15:  # Left side, tight
        return "left"
    elif pencil_number >= 16:  # Right side, tight
        return "right"
    else:
        return "unknown"

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
        rospy.logwarn("Gripper connection failed: {}.".format(e))

    rospy.loginfo("--- Keyboard Controls ---")
    rospy.loginfo(" +/- : Adjust active drawing height")
    rospy.loginfo(" O/C : Open/Close gripper")
    rospy.loginfo(" R   : Return current pencil to its holder")
    rospy.loginfo(" Q   : Quit (returns pencil first)")
    rospy.loginfo("Current workspace: {} ({}x{}m)".format(SELECTED_PAPER, PAPER_WIDTH, PAPER_HEIGHT))
    rospy.loginfo("Available colors are sent via TCP (e.g., #16160FB)")

    while not rospy.is_shutdown():
        c = getch()
        if c in ['r', 'R'] and current_pencil is not None:
            return_pencil_to_position(move_client, grasp_client, current_pencil)
        elif c in ['+', '=']:
            Z_ACTIVE = min(MAX_Z_HEIGHT, Z_ACTIVE + Z_INCREMENT)
            rospy.loginfo("Z_ACTIVE height adjusted to: {:.3f}".format(Z_ACTIVE))
            if pencil_ready and (time.time() - last_tcp_time <= 2.0):
                publish_pose(last_valid_x, last_valid_y, Z_ACTIVE)
        elif c in ['-', '_']:
            Z_ACTIVE = max(MIN_Z_HEIGHT, Z_ACTIVE - Z_INCREMENT)
            rospy.loginfo("Z_ACTIVE height adjusted to: {:.3f}".format(Z_ACTIVE))
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
            if current_pencil is not None:
                rospy.loginfo("Returning pencil before shutdown...")
                return_pencil_to_position(move_client, grasp_client, current_pencil)
            rospy.signal_shutdown("Keyboard quit")
            break


def tcp_receiver():
    global last_valid_x, last_valid_y, last_tcp_time, first_coordinate_after_idle

    move_client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
    grasp_client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)
    
    try:
        if move_client.wait_for_server(rospy.Duration(2.0)) and grasp_client.wait_for_server(rospy.Duration(2.0)):
            rospy.loginfo("Gripper clients ready for color changes.")
        else:
            rospy.logwarn("Gripper servers not available for color changes.")
    except Exception as e:
        rospy.logwarn("Gripper setup failed: {}".format(e))

    if sock is None:
        rospy.logerr("No TCP connection available.")
        return    
    
    try:
        rospy.loginfo("Connecting to TCP server at {}:{}...".format(SERVER_IP, SERVER_PORT))
        sock.connect((SERVER_IP, SERVER_PORT))
        rospy.loginfo("Connected. Receiving VR coordinates and color codes.")
        rospy.loginfo("Supported formats: 'x,y' or '#RRGGBB[s|b]'")
        
        sock.settimeout(1.0)
        
        buffer = b""
        coordinate_messages = 0
        
        while not rospy.is_shutdown():
            try:
                data = sock.recv(1024)
                if not data:
                    rospy.logwarn("Connection closed by server.")
                    break

                buffer += data
                
                while b"\n" in buffer:
                    line, buffer = buffer.split(b"\n", 1)
                    
                    try:
                        text = line.decode().strip()
                        if not text:
                            continue
                            
                        vr_x, vr_y, color_code = parse_tcp_message(text)
                        
                        if color_code is not None and vr_x is None and vr_y is None:
                            handle_color_change(color_code, move_client, grasp_client)
                            continue
                        
                        if vr_x is not None and vr_y is not None:
                            coordinate_messages += 1
                            
                            if color_code is not None:
                                handle_color_change(color_code, move_client, grasp_client)
                            
                            norm_x = vr_x / 1400.0
                            norm_y = vr_y / 1000.0
                            
                            if not (0.0 <= norm_x <= 1.0 and 0.0 <= norm_y <= 1.0):
                                if coordinate_messages % 180 == 0:
                                    rospy.logwarn("VR coordinates out of range: VR({}, {})".format(vr_x, vr_y))
                                continue
                            
                            raw_x, raw_y = map_normalized_to_workspace(norm_x, norm_y)
                            raw_x, raw_y = clamp_to_workspace_bounds(raw_x, raw_y)
                            
                            last_valid_x = raw_x
                            last_valid_y = raw_y
                            last_tcp_time = time.time()
                            
                            if pencil_ready:
                                # NEW LOGIC: Check if this is the first coordinate after idle
                                if first_coordinate_after_idle:
                                    rospy.loginfo("First coordinate after idle detected - using gradual approach")
                                    rospy.loginfo("first_coordinate_after_idle flag was: {}".format(first_coordinate_after_idle))
                                    gradual_approach_from_idle(raw_x, raw_y)
                                else:
                                    # Normal real-time following
                                    publish_pose(raw_x, raw_y, Z_ACTIVE)
                            else:
                                if coordinate_messages % 180 == 0:
                                    rospy.loginfo("Coordinates received but no pencil is selected. Use color codes to select a pencil.")
                        else:
                            rospy.logwarn("Invalid TCP message format: '{}'".format(text))
                            
                    except Exception as e:
                        rospy.logwarn("Parsing error: {} for line: '{}'".format(e, text))
                
            except socket.timeout:
                continue
            except Exception as e:
                rospy.logerr("TCP receive error: {}".format(e))
                break
                
    except Exception as e:
        rospy.logerr("TCP connection failed: {}".format(e))
    finally:
        sock.close()
        rospy.loginfo("TCP socket closed.")


def parse_tcp_message(message):
    """
    Parses the TCP message to extract coordinates or color code.
    """
    message = message.strip()
    
    if not message:
        return None, None, None

    parts = message.split()
    color_code = None
    coord_part = message
    
    for part in parts:
        if part.startswith('#') and len(part) == 8 and part[-1].lower() in ['s', 'b']:
            hex_part = part[:7].upper()
            suffix = part[7].lower()
            color_code = hex_part + suffix
            coord_part = message.replace(part, '').strip()
            break
    
    if color_code and not coord_part:
        return None, None, color_code
    
    if coord_part and ',' in coord_part:
        coord_parts = coord_part.split(',')
        if len(coord_parts) == 2:
            try:
                x = float(coord_parts[0].strip())
                y = float(coord_parts[1].strip())
                return x, y, color_code
            except ValueError:
                return None, None, color_code if color_code else None
    
    if message.startswith('#') and len(message) == 8 and message[-1].lower() in ['s', 'b']:
        hex_part = message[:7].upper()
        suffix = message[7].lower()
        return None, None, hex_part + suffix
    
    return None, None, None


def handle_color_change(color_code, move_client, grasp_client):
    """
    Handles color change based on the code received via TCP.
    """
    global current_pencil
    
    normalized_color_code = color_code[:7].upper() + color_code[7].lower()

    if normalized_color_code in COLOR_CODE_TO_PENCIL:
        new_pencil = COLOR_CODE_TO_PENCIL[normalized_color_code]
        
        if current_pencil == new_pencil:
            return
        
        rospy.loginfo("Color change requested: {} -> Pencil {}".format(normalized_color_code, new_pencil))
        
        if current_pencil is not None:
            rospy.loginfo("Returning current pencil {} before switching.".format(current_pencil))
            return_pencil_to_position(move_client, grasp_client, current_pencil)
        
        pickup_pencil_sequence(move_client, grasp_client, new_pencil)
        # Update Z_ACTIVE depending on pencil size
        global Z_ACTIVE
        if new_pencil <= 10:
            Z_ACTIVE = Z_ACTIVE_BIG
        else:
            Z_ACTIVE = Z_ACTIVE_SMALL
        rospy.loginfo("Z_ACTIVE set to {:.3f} after pencil change.".format(Z_ACTIVE))

        rospy.loginfo("Successfully switched to color {} (Pencil {})".format(normalized_color_code, new_pencil))
    else:
        rospy.logwarn("Unknown color code received: {}".format(color_code))


def idle_monitor():
    global CURRENT_Z_HEIGHT, first_coordinate_after_idle, gradual_approach_in_progress
    rate = rospy.Rate(2)
    already_idle = False
    
    while not rospy.is_shutdown():
        # Don't interfere if gradual approach is in progress
        if gradual_approach_in_progress:
            rate.sleep()
            continue
            
        if pencil_ready and (time.time() - last_tcp_time > 2.0):
            if not already_idle:
                CURRENT_Z_HEIGHT = Z_IDLE
                publish_pose(last_valid_x, last_valid_y, CURRENT_Z_HEIGHT)
                rospy.loginfo("Pencil moved to idle position: Z={} - Next coordinate will use gradual approach".format(Z_IDLE))
                already_idle = True
                first_coordinate_after_idle = True  # Flag that next coordinate needs gradual approach
        else:
            if already_idle and (time.time() - last_tcp_time <= 2.0):
                rospy.loginfo("Exiting idle state")
                already_idle = False
        
        rate.sleep()


def gradual_approach_from_idle(target_x, target_y):
    """
    Perform gradual approach from idle position to first drawing coordinate
    """
    global CURRENT_Z_HEIGHT, first_coordinate_after_idle, gradual_approach_in_progress
    
    rospy.loginfo("=== PERFORMING GRADUAL APPROACH FROM IDLE ===")
    rospy.loginfo("Target position: ({:.3f}, {:.3f})".format(target_x, target_y))
    
    # Set flag to prevent idle monitor interference
    gradual_approach_in_progress = True
    
    # Step 1: Move to target X,Y at current idle height
    rospy.loginfo("Step 1: Moving to target X,Y at idle height")
    publish_pose(target_x, target_y, Z_IDLE)
    time.sleep(0.8)  # Wait for movement to complete
    
    # Step 2: Gradually descend to active drawing height
    rospy.loginfo("Step 2: Gradually descending to active height")
    perform_gradual_movement(
        target_x, target_y, Z_IDLE,
        target_x, target_y, Z_ACTIVE,
        steps=6, delay=0.3
    )
    
    CURRENT_Z_HEIGHT = Z_ACTIVE
    first_coordinate_after_idle = False
    gradual_approach_in_progress = False  # Clear flag
    rospy.loginfo("=== GRADUAL APPROACH COMPLETE - READY FOR RT FOLLOWING ===")
    rospy.loginfo("first_coordinate_after_idle flag set to: {}".format(first_coordinate_after_idle))


def main():
    global pub, gripper_available, first_coordinate_after_idle, sock
    rospy.init_node("tcp_drawing_robot_control")
    pub = rospy.Publisher("/demo/pose_final", PoseStamped, queue_size=10)


    # Initialize socket connection at startup
    sock = None
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Allow socket reuse
        rospy.loginfo("Connecting to TCP server at {}:{}...".format(SERVER_IP, SERVER_PORT))
        sock.connect((SERVER_IP, SERVER_PORT))
        rospy.loginfo("TCP connection established.")
        sock.settimeout(1.0)
    except socket.error as e:
        if e.errno == 106:  # Already connected
            rospy.logwarn("Socket already connected, continuing...")
        else:
            rospy.logerr("TCP connection failed: {}".format(e))
            if sock:
                sock.close()
            sock = None
    except Exception as e:
        rospy.logerr("TCP connection failed: {}".format(e))
        if sock:
            sock.close()
        sock = None

        
    # Initialize flags
    first_coordinate_after_idle = True
    gradual_approach_in_progress = False

    # Pick up default pencil after publisher is created
    move_client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
    grasp_client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)

    rospy.loginfo("Waiting for gripper servers to become available for default pencil pickup...")
    if move_client.wait_for_server(rospy.Duration(5.0)) and grasp_client.wait_for_server(rospy.Duration(5.0)):
        rospy.loginfo("Gripper servers ready. Picking up default pencil...")
        gripper_available = True  # This now properly sets the global variable
        pickup_pencil_sequence(move_client, grasp_client, 1)
    else:
        rospy.logwarn("Gripper servers not available. Skipping default pencil pickup.")
    
    rospy.loginfo("=== WORKSPACE CONFIGURATION ===")
    rospy.loginfo("Paper size: {}".format(SELECTED_PAPER))
    rospy.loginfo("Dimensions: {}m x {}m".format(PAPER_WIDTH, PAPER_HEIGHT))
    rospy.loginfo("Center: ({}, {})".format(WORKSPACE_CENTER_X, WORKSPACE_CENTER_Y))
    rospy.loginfo("FR3 X range: {:.3f} to {:.3f}".format(WORKSPACE_X_MIN, WORKSPACE_X_MAX))
    rospy.loginfo("FR3 Y range: {:.3f} to {:.3f}".format(WORKSPACE_Y_MIN, WORKSPACE_Y_MAX))
    rospy.loginfo("Coordinate mapping: R2_X->FR3_Y, R2_Y->FR3_X")
    rospy.loginfo("Coordinate flips: X={}, Y={}".format(FLIP_X, FLIP_Y))
    rospy.loginfo("===============================")
    
    threading.Thread(target=keyboard_listener, daemon=True).start()
    threading.Thread(target=tcp_receiver, daemon=True).start()
    threading.Thread(target=idle_monitor, daemon=True).start()
    
    rospy.spin()

if __name__ == "__main__":
    main()