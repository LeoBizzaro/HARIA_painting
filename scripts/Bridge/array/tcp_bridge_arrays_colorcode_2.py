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

# === TCP Configuration ===
SERVER_IP = "127.0.0.1"
SERVER_PORT = 5005

# === Workspace Configuration ===
PAPER_SIZES = {
    "A2": (0.420, 0.594),
    "A3": (0.297, 0.420),
    "A4": (0.210, 0.297),
    "A5": (0.148, 0.210)
}

SELECTED_PAPER = "A3"
PAPER_WIDTH, PAPER_HEIGHT = PAPER_SIZES[SELECTED_PAPER]

WORKSPACE_CENTER_X = 0.5
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
MIN_Z_HEIGHT = 0.09
MAX_Z_HEIGHT = 0.25
Z_ACTIVE = 0.125  # Drawing height
Z_IDLE = 0.140    # Idle height for movement
Z_LIFT = 0.200    # Safe height for movement over pencils
Z_APPROACH_OFFSET = 0.05 # Offset to approach from above
PENCIL_BASE_Z = 0.125 # Height where pencils are picked/dropped

# === Gripper Configuration ===
GRIPPER_OPEN_WIDTH = 0.08  # Width for fully open gripper
GRIPPER_CLOSE_WIDTH = 0.015 # Width for grasping a pencil
GRIPPER_GRASP_FORCE = 40  # Force for grasping

# === State and ROS Globals ===
pub = None
current_z_height = Z_IDLE
current_x = WORKSPACE_CENTER_X
current_y = WORKSPACE_CENTER_Y
current_pencil_id = None
gripper_available = False
move_client = None
grasp_client = None

# === Pencil Rack Positions ===
PENCIL_POSITIONS = {
    1: {"x": 0.35, "y": -0.22, "angle": 0.0},
    2: {"x": 0.35, "y": -0.24, "angle": 0.0},
    3: {"x": 0.35, "y": -0.26, "angle": 0.0},
    4: {"x": 0.35, "y": -0.28, "angle": 0.0},
    5: {"x": 0.35, "y": -0.30, "angle": 0.0},
    6: {"x": 0.35, "y": -0.32, "angle": 0.0},
    7: {"x": 0.35, "y": -0.34, "angle": 0.0},
    8: {"x": 0.35, "y": -0.36, "angle": 0.0},
    9: {"x": 0.35, "y": -0.38, "angle": 0.0},
    10: {"x": 0.35, "y": -0.40, "angle": 0.0},
}


def send_pose(x, y, z):
    """
    Publishes a PoseStamped message to the robot's control topic.
    The message contains the position (x, y, z) and a fixed orientation.
    """
    global pub
    
    pose_msg = PoseStamped()
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.header.frame_id = "fr3_link0" # Ensure the correct frame
    
    # Position
    pose_msg.pose.position.x = x
    pose_msg.pose.position.y = y
    pose_msg.pose.position.z = z
    
    # Orientation (fixed)
    # This orientation seems to be correct for the Franka's end-effector
    # pointing straight down.
    pose_msg.pose.orientation.x = 1.0
    pose_msg.pose.orientation.y = 0.0
    pose_msg.pose.orientation.z = 0.0
    pose_msg.pose.orientation.w = 0.0
    
    pub.publish(pose_msg)
    
def send_move_goal(width, speed=0.1):
    """Sends a Move goal to the gripper."""
    global gripper_available, move_client
    if not gripper_available:
        rospy.logwarn("Gripper not available, cannot send move goal.")
        return
    
    goal = MoveGoal()
    goal.width = width
    goal.speed = speed
    move_client.send_goal(goal)
    move_client.wait_for_result()
    rospy.loginfo(f"Gripper moved to {width:.3f}m")

def send_grasp_goal(width, force=40):
    """Sends a Grasp goal to the gripper."""
    global gripper_available, grasp_client
    if not gripper_available:
        rospy.logwarn("Gripper not available, cannot send grasp goal.")
        return
    
    goal = GraspGoal()
    goal.width = width
    goal.force = force
    goal.epsilon.inner = 0.005 # Default from Franka examples
    goal.epsilon.outer = 0.005 # Default from Franka examples
    grasp_client.send_goal(goal)
    grasp_client.wait_for_result()
    rospy.loginfo(f"Gripper grasped with width {width:.3f}m and force {force}N")

def drop_pencil(pencil_id):
    """
    Drops a pencil at its designated position.
    
    Args:
        pencil_id (int): The ID of the pencil to drop (1-10).
    """
    if pencil_id in PENCIL_POSITIONS:
        pos = PENCIL_POSITIONS[pencil_id]
        
        # 1. Lift to a safe height
        rospy.loginfo(f"Dropping pencil {pencil_id}: lifting to {Z_LIFT:.3f}m")
        send_pose(pos["x"], pos["y"], Z_LIFT)
        
        # 2. Move to the drop position
        rospy.loginfo(f"Moving to drop position: ({pos['x']:.3f}, {pos['y']:.3f})")
        # This is a redundant move, but good for safety to ensure a valid pose is sent
        send_pose(pos["x"], pos["y"], Z_LIFT) 
        
        # 3. Lower to the drop height (PENCIL_BASE_Z)
        rospy.loginfo(f"Lowering to drop height: {PENCIL_BASE_Z:.3f}m")
        send_pose(pos["x"], pos["y"], PENCIL_BASE_Z)
        
        # 4. Open the gripper to release the pencil
        rospy.loginfo("Opening gripper to release pencil")
        send_move_goal(GRIPPER_OPEN_WIDTH)
        
        # 5. Lift up again to a safe height
        rospy.loginfo("Lifting up after dropping")
        send_pose(pos["x"], pos["y"], Z_LIFT)
    else:
        rospy.logwarn(f"Pencil ID {pencil_id} not found in PENCIL_POSITIONS.")

def pick_pencil(pencil_id):
    """
    Picks up a pencil from its designated position.
    
    Args:
        pencil_id (int): The ID of the pencil to pick (1-10).
    """
    if pencil_id in PENCIL_POSITIONS:
        pos = PENCIL_POSITIONS[pencil_id]
        
        # 1. Open gripper and lift to safe height before moving
        rospy.loginfo(f"Picking up pencil {pencil_id}: Opening gripper and lifting to {Z_LIFT:.3f}m")
        send_move_goal(GRIPPER_OPEN_WIDTH) # Open gripper
        send_pose(pos["x"], pos["y"], Z_LIFT)
        
        # 2. Move above the pencil
        rospy.loginfo(f"Moving above pencil {pencil_id} at ({pos['x']:.3f}, {pos['y']:.3f})")
        send_pose(pos["x"], pos["y"], Z_LIFT) # Move above the pencil location
        
        # 3. Lower to the pickup height (PENCIL_BASE_Z)
        rospy.loginfo(f"Lowering to pickup height: {PENCIL_BASE_Z:.3f}m")
        send_pose(pos["x"], pos["y"], PENCIL_BASE_Z)
        
        # 4. Grasp the pencil
        rospy.loginfo("Closing gripper to grasp pencil")
        send_grasp_goal(GRIPPER_CLOSE_WIDTH, GRIPPER_GRASP_FORCE)
        
        # 5. Lift up again to a safe height after grasping
        rospy.loginfo("Lifting up after grasping")
        send_pose(pos["x"], pos["y"], Z_LIFT)
        
    else:
        rospy.logwarn(f"Pencil ID {pencil_id} not found in PENCIL_POSITIONS.")

def tcp_connection_handler(conn, addr):
    """
    Handles a single TCP client connection.
    """
    global current_z_height, current_x, current_y, current_pencil_id
    rospy.loginfo(f"Connected by {addr}")
    try:
        while not rospy.is_shutdown():
            data = conn.recv(1024).decode('utf-8')
            if not data:
                break
            
            # Process received data
            rospy.loginfo(f"Received from client: {data.strip()}")
            parts = data.strip().split(',')
            msg_type = parts[0].upper()
            
            if msg_type == "POSE":
                try:
                    # R2 coordinates are 2D (x, y) from 0 to 1
                    r2_x_norm = float(parts[1])
                    r2_y_norm = float(parts[2])
                    
                    # Convert normalized R2 coords to FR3 workspace coords
                    fr3_x = WORKSPACE_X_MIN + (r2_x_norm * PAPER_WIDTH)
                    fr3_y = WORKSPACE_Y_MIN + (r2_y_norm * PAPER_HEIGHT)
                    
                    # Apply flips
                    if FLIP_X:
                        fr3_x = WORKSPACE_X_MAX - (r2_x_norm * PAPER_WIDTH)
                    if FLIP_Y:
                        fr3_y = WORKSPACE_Y_MAX - (r2_y_norm * PAPER_HEIGHT)
                        
                    # Update current position
                    current_x = fr3_x
                    current_y = fr3_y
                    current_z_height = Z_ACTIVE
                    
                    # Send the pose to the robot
                    send_pose(current_x, current_y, current_z_height)
                    rospy.loginfo(f"Moving to: ({current_x:.3f}, {current_y:.3f}, {current_z_height:.3f})")

                except (ValueError, IndexError) as e:
                    rospy.logwarn(f"Invalid POSE command format: {data}. Error: {e}")
            
            elif msg_type == "PICK":
                try:
                    pencil_id = int(parts[1])
                    if pencil_id != current_pencil_id:
                        # Drop the current pencil first
                        if current_pencil_id is not None:
                            drop_pencil(current_pencil_id)
                        
                        # Then pick the new one
                        pick_pencil(pencil_id)
                        current_pencil_id = pencil_id
                        rospy.loginfo(f"Pencil {pencil_id} picked.")
                    else:
                        rospy.loginfo(f"Pencil {pencil_id} is already in the gripper.")
                except (ValueError, IndexError) as e:
                    rospy.logwarn(f"Invalid PICK command format: {data}. Error: {e}")

            elif msg_type == "LIFT":
                # Lift the pen to the idle height
                current_z_height = Z_IDLE
                send_pose(current_x, current_y, current_z_height)
                rospy.loginfo(f"Lifting pen to Z={current_z_height:.3f}m")
            
            elif msg_type == "DROP":
                # Drop the pen to the drawing height
                current_z_height = Z_ACTIVE
                send_pose(current_x, current_y, current_z_height)
                rospy.loginfo(f"Dropping pen to Z={current_z_height:.3f}m")
                
            elif msg_type == "Z_UP":
                current_z_height = min(current_z_height + 0.005, MAX_Z_HEIGHT)
                send_pose(current_x, current_y, current_z_height)
                rospy.loginfo(f"Z_height increased to: {current_z_height:.3f}m")
            
            elif msg_type == "Z_DOWN":
                current_z_height = max(current_z_height - 0.005, MIN_Z_HEIGHT)
                send_pose(current_x, current_y, current_z_height)
                rospy.loginfo(f"Z_height decreased to: {current_z_height:.3f}m")
                
            elif msg_type == "CLOSE_GRIPPER":
                send_grasp_goal(GRIPPER_CLOSE_WIDTH, GRIPPER_GRASP_FORCE)
            
            elif msg_type == "OPEN_GRIPPER":
                send_move_goal(GRIPPER_OPEN_WIDTH)
            
            else:
                rospy.logwarn(f"Unknown command: {data}")

    finally:
        rospy.loginfo(f"Client {addr} disconnected.")
        conn.close()

def tcp_server_thread():
    """
    Main TCP server loop that accepts incoming connections.
    """
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((SERVER_IP, SERVER_PORT))
        s.listen()
        rospy.loginfo(f"TCP server listening on {SERVER_IP}:{SERVER_PORT}")
        
        while not rospy.is_shutdown():
            try:
                conn, addr = s.accept()
                client_thread = threading.Thread(target=tcp_connection_handler, args=(conn, addr))
                client_thread.daemon = True
                client_thread.start()
            except socket.error as e:
                rospy.logerr(f"Socket error accepting connection: {e}")
                # Re-listen after a short delay
                time.sleep(1)
            except Exception as e:
                rospy.logerr(f"An unexpected error occurred in the server thread: {e}")
                
def initialize_gripper_clients():
    """
    Initializes the action clients for the Franka gripper.
    """
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

def main():
    global pub
    
    rospy.init_node("simplified_tcp_drawing")
    pub = rospy.Publisher("/demo/pose_final", PoseStamped, queue_size=10)
    
    # Log configuration
    rospy.loginfo(f"=== SIMPLIFIED TCP DRAWING BRIDGE ===")
    rospy.loginfo(f"Paper: {SELECTED_PAPER} ({PAPER_WIDTH}x{PAPER_HEIGHT}m)")
    rospy.loginfo(f"Workspace center: ({WORKSPACE_CENTER_X}, {WORKSPACE_CENTER_Y})")
    rospy.loginfo(f"Drawing height (Z_ACTIVE): {Z_ACTIVE:.3f}m")
    rospy.loginfo(f"Pencil pickup height: {PENCIL_BASE_Z:.3f}m")
    rospy.loginfo(f"Controls: TCP commands from client")
    rospy.loginfo(f"=======================================")
    
    # Initialize gripper
    initialize_gripper_clients()
    
    # Start the TCP server in a separate thread
    tcp_server_thread_obj = threading.Thread(target=tcp_server_thread)
    tcp_server_thread_obj.daemon = True
    tcp_server_thread_obj.start()
    
    rospy.spin()

if __name__ == "__main__":
    main()