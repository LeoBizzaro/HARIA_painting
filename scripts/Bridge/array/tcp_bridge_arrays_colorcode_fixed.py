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
SELECTED_PAPER = "A3"
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
MIN_Z_HEIGHT = 0.09
MAX_Z_HEIGHT = 0.25
Z_ACTIVE = 0.13  # This will be adjustable with +/- keys
Z_IDLE = 0.14
Z_LIFT = 0.200  # Height to lift between arrays
CURRENT_Z_HEIGHT = Z_IDLE
Z_INCREMENT = 0.001  # Fine increment for Z_ACTIVE adjustments

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

current_pencil = None  # Currently selected pencil (1, 2, or 3)

# === COLOR HANDLING ===  TO BE UPDATED
COLOR_CODE_TO_PENCIL = {
    "#16160F": 1,  # Black
    "#7A3D28": 2,  # Brown
    "#D82929": 3,  # Red
    "#E86E09": 4,  # Orange
    "#DBC416": 5,  # Yellow
    "#49C893": 6,  # Light Green
    "#02704D": 7,  # Dark Green
    "#0D2875": 8,  # Dark Blue
    "#0295D5": 9,  # Light Blue
    "#C0BDAE": 10,  # White
}

current_color_code = None  # Codice colore attuale (es: "#FF7200")
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


def pickup_pencil_sequence(move_client, grasp_client, pencil_number):
    """
    Pick up a specific pencil/color
    Uses Z-offset approach for safe movement with intermediate safety position
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
    
    rospy.loginfo(f"Starting pickup sequence for {pencil_pos['name']} with safe movement...")
    
    # Step 1: Open gripper fully
    rospy.loginfo("Opening gripper...")
    if gripper_available:
        move_goal = MoveGoal(width=0.05, speed=0.1)
        move_client.send_goal(move_goal)
        move_client.wait_for_result()
    gripper_closed = False
    time.sleep(1.0)
    
    # CORREZIONE: Aggiunta di posizione intermedia sicura per evitare emergency distance
    # Determina se il movimento è da sinistra a destra o viceversa
    current_y = last_valid_y
    target_y = pencil_pos['y']
    
    # Posizione intermedia sicura al centro del workspace
    safe_intermediate_x = WORKSPACE_CENTER_X
    safe_intermediate_y = WORKSPACE_CENTER_Y
    safe_intermediate_z = max(Z_LIFT + 0.05, 0.25)  # Altezza extra per sicurezza
    
    # Verifica se è necessario un movimento intermedio (differenza Y > 20cm)
    y_distance = abs(target_y - current_y)
    needs_intermediate_move = y_distance > 0.20  # 20cm threshold
    
    if needs_intermediate_move:
        rospy.loginfo(f"Large Y distance detected ({y_distance:.3f}m). Using intermediate safe position...")
        
        # Step 2a: Muovi prima in una posizione intermedia sicura (MOVIMENTO VELOCE)
        rospy.loginfo("Moving to intermediate safe position...")
        perform_gradual_movement(
            last_valid_x, last_valid_y, CURRENT_Z_HEIGHT,
            safe_intermediate_x, safe_intermediate_y, safe_intermediate_z,
            steps=3, delay=0.5
        )
        
        # Aggiorna la posizione corrente
        last_valid_x = safe_intermediate_x
        last_valid_y = safe_intermediate_y
        CURRENT_Z_HEIGHT = safe_intermediate_z
        
        # Pausa per stabilizzazione
        time.sleep(0.5)
        
        # Step 2b: Muovi dalla posizione intermedia alla posizione di approach della matita (MOVIMENTO LENTO E PRECISO)
        rospy.loginfo(f"Moving from safe position to approach position above {pencil_pos['name']}...")
        # MODIFICA: Ripristinato movimento lento per la fase di approccio
        perform_gradual_movement(
            safe_intermediate_x, safe_intermediate_y, safe_intermediate_z,
            pencil_pos['x'], pencil_pos['y'], approach_z,
            steps=8, delay=0.7
        )
    else:
        # Step 2: Movimento diretto (quando la distanza è piccola)
        rospy.loginfo(f"Direct movement to approach position above {pencil_pos['name']}...")
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
    # publish_pose(pencil_pos['x'], pencil_pos['y'], approach_z)
    perform_gradual_movement(
        pencil_pos['x'], pencil_pos['y'], pencil_pos['z'],
        pencil_pos['x'], pencil_pos['y'], approach_z,
        steps=3, delay=0.4
    )
    time.sleep(0.8)
    
    # Step 7: Move to drawing position gradually
    # Se abbiamo usato una posizione intermedia, torniamo tramite quella
    if needs_intermediate_move:
        rospy.loginfo("Returning to safe intermediate position with pencil... (MOVIMENTO VELOCE)")
        perform_gradual_movement(
            pencil_pos['x'], pencil_pos['y'], approach_z,
            safe_intermediate_x, safe_intermediate_y, safe_intermediate_z,
            steps=3, delay=0.5
        )
        
        rospy.loginfo("Moving from safe position to drawing area... (MOVIMENTO VELOCE)")
        perform_gradual_movement(
            safe_intermediate_x, safe_intermediate_y, safe_intermediate_z,
            WORKSPACE_CENTER_X, WORKSPACE_CENTER_Y, Z_IDLE,
            steps=3, delay=0.5
        )
        
        # Aggiorna la posizione finale
        last_valid_x = WORKSPACE_CENTER_X
        last_valid_y = WORKSPACE_CENTER_Y
    else:
        # Movimento diretto alla posizione di disegno
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
    
    rospy.loginfo(f"{pencil_pos['name']} pickup complete with safe movement. Robot is ready for drawing.")


def return_pencil_to_position(move_client, grasp_client, pencil_number):
    """
    Return the current pencil to its designated position
    Uses Z-offset approach for safe movement with intermediate position
    SIMPLIFIED: Always assumes success
    """
    global gripper_closed, current_pencil, pencil_ready, CURRENT_Z_HEIGHT, last_valid_x, last_valid_y
    
    if pencil_number not in PENCIL_POSITIONS:
        rospy.logwarn(f"Invalid pencil number: {pencil_number}")
        return
    
    pencil_pos = PENCIL_POSITIONS[pencil_number]
    approach_z = pencil_pos['z'] + Z_APPROACH_OFFSET
    
    rospy.loginfo(f"Returning {pencil_pos['name']} to position with safe movement...")
    
    # CORREZIONE: Aggiunta di movimento intermedio anche per il ritorno
    current_y = last_valid_y
    target_y = pencil_pos['y']
    
    # Posizione intermedia sicura
    safe_intermediate_x = WORKSPACE_CENTER_X
    safe_intermediate_y = WORKSPACE_CENTER_Y
    safe_intermediate_z = max(Z_LIFT + 0.05, 0.25)
    
    # Verifica se è necessario un movimento intermedio
    y_distance = abs(target_y - current_y)
    needs_intermediate_move = y_distance > 0.20  # 20cm threshold
    
    if needs_intermediate_move:
        rospy.loginfo(f"Large return distance detected ({y_distance:.3f}m). Using intermediate safe position...")
        
        # Step 1: Muovi alla posizione intermedia sicura (MOVIMENTO VELOCE)
        rospy.loginfo("Moving to intermediate safe position for return...")
        perform_gradual_movement(
            last_valid_x, last_valid_y, CURRENT_Z_HEIGHT,
            safe_intermediate_x, safe_intermediate_y, safe_intermediate_z,
            steps=3, delay=0.5
        )
        
        # Step 2: Dalla posizione intermedia alla posizione di approach della matita (MOVIMENTO LENTO E PRECISO)
        rospy.loginfo(f"Moving from safe position to approach position above {pencil_pos['name']}...")
        # MODIFICA: Ripristinato movimento lento per la fase di approccio
        perform_gradual_movement(
            safe_intermediate_x, safe_intermediate_y, safe_intermediate_z,
            pencil_pos['x'], pencil_pos['y'], approach_z,
            steps=6, delay=0.7
        )
    else:
        # Step 1: Movimento diretto alla posizione di approach
        rospy.loginfo(f"Direct movement to approach position above {pencil_pos['name']}...")
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
    
    # SIMPLIFIED: Always consider it successful
    current_pencil = None
    pencil_ready = False
    
    rospy.loginfo(f"{pencil_pos['name']} returned successfully with safe movement.")

# def process_coordinate_array(coordinates, color_changed=False):
#     """
#     Process an array of coordinates by moving through them sequentially
#     Note: Pencil selection is now handled automatically in tcp_receiver
    
#     Args:
#         coordinates: List of (x, y) coordinate tuples
#         color_changed: Boolean indicating if this array follows a color change
#     """
#     global last_valid_x, last_valid_y, CURRENT_Z_HEIGHT, processing_array, array_count
    
#     if len(coordinates) == 0:
#         rospy.logwarn("Received empty coordinate array")
#         return
    
#     # Note: Removed pencil_ready check as pencil selection is now automatic
#     processing_array = True
#     array_count += 1
    
#     rospy.loginfo(f"=== PROCESSING ARRAY #{array_count} with {len(coordinates)} coordinates ===")
#     if color_changed:
#         rospy.loginfo("Color change detected - using safe intermediate movement")
    
#     # Step 1: Lift the end effector
#     rospy.loginfo("Lifting end effector before array execution...")
#     publish_pose(last_valid_x, last_valid_y, Z_LIFT)
#     time.sleep(0.5)
    
#     # Step 2: If color changed, move to center first for safe transition
#     first_x, first_y = coordinates[0]
#     if color_changed:
#         # Calculate distance from current position to first coordinate
#         distance = math.sqrt((first_x - last_valid_x)**2 + (first_y - last_valid_y)**2)
        
#         # If distance is significant (more than half the workspace), use center waypoint
#         workspace_diagonal = math.sqrt(PAPER_WIDTH**2 + PAPER_HEIGHT**2)
#         if distance > workspace_diagonal * 0.3:  # 30% of diagonal
#             rospy.loginfo(f"Large movement detected ({distance:.3f}m). Moving via center waypoint...")
            
#             # Move to workspace center at lift height
#             center_x = WORKSPACE_CENTER_X
#             center_y = WORKSPACE_CENTER_Y
#             rospy.loginfo(f"Moving to center waypoint: ({center_x:.3f}, {center_y:.3f})")
#             publish_pose(center_x, center_y, Z_LIFT)
#             time.sleep(0.8)  # Longer pause at center
            
#             # Update last position to center
#             last_valid_x = center_x
#             last_valid_y = center_y
    
#     # Step 3: Move to the first coordinate of the array (at lift height)
#     rospy.loginfo(f"Moving to first coordinate: ({first_x:.3f}, {first_y:.3f}) at lift height")
#     publish_pose(first_x, first_y, Z_LIFT)
#     time.sleep(0.3)
    
#     # Step 4: Lower to active drawing height
#     rospy.loginfo("Lowering to drawing height...")
#     publish_pose(first_x, first_y, Z_ACTIVE)
#     time.sleep(0.3)
    
#     # Step 5: Execute the array by moving through all coordinates
#     rospy.loginfo("Executing coordinate array...")
#     for i, (x, y) in enumerate(coordinates):
#         publish_pose(x, y, Z_ACTIVE)
#         last_valid_x = x
#         last_valid_y = y
        
#         # Small delay between points to ensure smooth movement
#         time.sleep(0.05)  # 50ms delay - adjust as needed
        
#         # Log progress periodically
#         if (i + 1) % max(1, len(coordinates) // 10) == 0:
#             progress = ((i + 1) / len(coordinates)) * 100
#             rospy.loginfo(f"Array progress: {i+1}/{len(coordinates)} ({progress:.1f}%)")
    
#     # Step 6: Lift the end effector after completing the array
#     rospy.loginfo("Array complete. Lifting end effector...")
#     publish_pose(last_valid_x, last_valid_y, Z_LIFT)
#     time.sleep(0.3)
    
#     # Update current height
#     CURRENT_Z_HEIGHT = Z_LIFT
    
#     processing_array = False
#     rospy.loginfo(f"=== ARRAY #{array_count} COMPLETED ===")

def process_coordinate_array(coordinates, color_changed=False):
    """
    Process an array of coordinates by moving through them sequentially
    Note: Pencil selection is now handled automatically in tcp_receiver
    
    Args:
        coordinates: List of (x, y) coordinate tuples
        color_changed: Boolean indicating if this array follows a color change
    """
    global last_valid_x, last_valid_y, CURRENT_Z_HEIGHT, processing_array, array_count
    
    if len(coordinates) == 0:
        rospy.logwarn("Received empty coordinate array")
        return
    
    # Note: Removed pencil_ready check as pencil selection is now automatic
    processing_array = True
    array_count += 1
    
    rospy.loginfo(f"=== PROCESSING ARRAY #{array_count} with {len(coordinates)} coordinates ===")
    if color_changed:
        rospy.loginfo("Color change detected - using safe intermediate movement")
    
    # Step 1: Lift the end effector
    rospy.loginfo("Lifting end effector before array execution...")
    publish_pose(last_valid_x, last_valid_y, Z_LIFT)
    time.sleep(0.5)
    
    # Step 2: If color changed, move to center first for safe transition
    first_x, first_y = coordinates[0]
    if color_changed:
        # Calculate distance from current position to first coordinate
        distance = math.sqrt((first_x - last_valid_x)**2 + (first_y - last_valid_y)**2)
        
        # If distance is significant (more than half the workspace), use center waypoint
        workspace_diagonal = math.sqrt(PAPER_WIDTH**2 + PAPER_HEIGHT**2)
        if distance > workspace_diagonal * 0.3:  # 30% of diagonal
            rospy.loginfo(f"Large movement detected ({distance:.3f}m). Moving via center waypoint...")
            
            # Move to workspace center at lift height
            center_x = WORKSPACE_CENTER_X
            center_y = WORKSPACE_CENTER_Y
            rospy.loginfo(f"Moving to center waypoint: ({center_x:.3f}, {center_y:.3f})")
            publish_pose(center_x, center_y, Z_LIFT)
            time.sleep(0.8)  # Longer pause at center
            
            # Update last position to center
            last_valid_x = center_x
            last_valid_y = center_y
    
    # Step 3: Move to the first coordinate of the array (at lift height)
    rospy.loginfo(f"Moving to first coordinate: ({first_x:.3f}, {first_y:.3f}) at lift height")
    publish_pose(first_x, first_y, Z_LIFT)
    time.sleep(0.3)
    
    # Step 4: Lower to active drawing height GRADUALLY
    rospy.loginfo("Lowering to drawing height gradually...")
    # --- MODIFICATION START ---
    # OLD CODE (REMOVED):
    # publish_pose(first_x, first_y, Z_ACTIVE)
    # time.sleep(0.3)
    # NEW CODE (ADDED):
    perform_gradual_movement(
        first_x, first_y, Z_LIFT,      # Start position
        first_x, first_y, Z_ACTIVE,    # End position
        steps=5, delay=0.3             # Control the speed of the descent
    )
    # --- MODIFICATION END ---
    
    # Step 5: Execute the array by moving through all coordinates
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
    
    # Step 6: Lift the end effector after completing the array
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
    rospy.loginfo(f"Colors: 1/2/3 = manual pencil selection (optional), R = return current pencil")
    rospy.loginfo(f"Arrays: I = show status")
    rospy.loginfo(f"Current workspace: {SELECTED_PAPER} paper ({PAPER_WIDTH}x{PAPER_HEIGHT}m)")
    rospy.loginfo(f"AUTOMATIC MODE: Pencil selection is automatic based on color codes received")
    rospy.loginfo(f"First color code received will automatically select the corresponding pencil")
    
    # Rest of the keyboard_listener function remains the same...
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
    Each line contains an array of coordinates: "#COLORCODE x1,y1 x2,y2 x3,y3 ..."
    """
    global processing_array, current_color_code

    # Initialize gripper clients for pencil changes
    move_client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
    grasp_client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        rospy.loginfo(f"Connecting to TCP server at {SERVER_IP}:{SERVER_PORT}...")
        sock.connect((SERVER_IP, SERVER_PORT))
        rospy.loginfo("Connected. Receiving coordinate arrays.")
        rospy.loginfo("Expected format: '#COLORCODE x1,y1 x2,y2 x3,y3 ...' (one array per line)")
        rospy.loginfo("Color code at the beginning of each line")

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
                        if not text:
                            continue

                        # Extract color code at the beginning (e.g., "#FF0000")
                        color_code = None
                        coords_text = text
                        
                        # Look for color code pattern at the beginning
                        parts = text.split()
                        if len(parts) > 0 and parts[0].startswith('#') and len(parts[0]) == 7:
                            color_code = parts[0].upper()  # Normalize to uppercase
                            coords_text = ' '.join(parts[1:])  # Remove color code from coordinates
                            rospy.loginfo(f"Received array #{array_count_received}, color: {color_code}")
                        else:
                            rospy.loginfo(f"Received array #{array_count_received}, no color specified")

                        # Parse coordinates
                        coordinates = parse_coordinate_line(coords_text)
                        if len(coordinates) == 0:
                            rospy.logwarn(f"No valid coordinates found in array #{array_count_received}")
                            continue

                        # Track if color is changing for this array
                        color_changed = False

                        # Handle pencil selection automatically based on color code
                        if color_code:
                            pencil_number = COLOR_CODE_TO_PENCIL.get(color_code)
                            if pencil_number:
                                # Change pencil if color is different from current OR if no pencil is selected yet
                                if color_code != current_color_code or current_pencil is None:
                                    color_changed = True  # Mark that color is changing
                                    
                                    if current_pencil is None:
                                        rospy.loginfo(f"First color received: {color_code}. Automatically selecting pencil {pencil_number}")
                                    else:
                                        rospy.loginfo(f"Color changed from {current_color_code} to {color_code}")
                                    
                                    rospy.loginfo(f"Switching to pencil {pencil_number} for color {color_code}")
                                    
                                    # Wait for any current array processing to complete before changing pencil
                                    while processing_array and not rospy.is_shutdown():
                                        rospy.loginfo("Waiting for current array to complete before pencil change...")
                                        time.sleep(0.1)
                                    
                                    pickup_pencil_sequence(move_client, grasp_client, pencil_number)
                                    current_color_code = color_code
                                    rospy.loginfo(f"Pencil changed successfully to color {color_code}")
                                else:
                                    rospy.loginfo(f"Color {color_code} already active, no pencil change needed")
                            else:
                                rospy.logwarn(f"Color {color_code} not recognized. Available colors: {list(COLOR_CODE_TO_PENCIL.keys())}")
                                if current_pencil is None:
                                    rospy.logwarn("No pencil selected and color not recognized. Cannot proceed.")
                                    continue
                        else:
                            if current_pencil is None:
                                rospy.logwarn("No color specified and no pencil selected. Cannot proceed without color information.")
                                continue
                            else:
                                rospy.loginfo(f"No color specified, continuing with current color {current_color_code}")

                        # Wait for any current array processing to complete
                        while processing_array and not rospy.is_shutdown():
                            rospy.loginfo("Waiting for current array to complete...")
                            time.sleep(0.1)

                        # Process the coordinate array with color change information
                        rospy.loginfo(f"Processing {len(coordinates)} coordinates from array #{array_count_received}")
                        process_coordinate_array(coordinates, color_changed)

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