#!/usr/bin/env python3
"""
Simplified ROS node to control a Franka Emika robot for drawing.

This script listens for TCP connections to receive drawing commands.
It processes arrays of coordinates, handles automatic pencil changes based on
color codes, and controls the robot's movement to draw on a defined workspace.

Author: Gemini
Date: 25 June 2025
"""

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

# ==============================================================================
# --- CONFIGURATION PARAMETERS ---
# ==============================================================================

# --- TCP Server Configuration ---
SERVER_IP = "127.0.0.1"  # IP to connect to (e.g., '192.168.1.111' or '127.0.0.1' for localhost)
SERVER_PORT = 5005      # Port to connect to

# --- Workspace Configuration ---
PAPER_SIZES = {
    "A3": (0.297, 0.420), # width x height in meters
    "A4": (0.210, 0.297),
}
SELECTED_PAPER = "A3"
PAPER_WIDTH, PAPER_HEIGHT = PAPER_SIZES[SELECTED_PAPER]

# --- Robot Physical Workspace ---
WORKSPACE_CENTER_X = 0.5
WORKSPACE_CENTER_Y = 0.0
# NOTE: The mapping from canvas to robot coordinates is handled later.
# These flags control the orientation of the drawing on the paper.
FLIP_X = True  # Flips the drawing horizontally
FLIP_Y = True  # Flips the drawing vertically

# --- Robot Movement & Pose ---
Z_IDLE = 0.14           # Z-height when moving between points (not drawing)
Z_LIFT = 0.20           # Higher Z-height for safe long-distance moves
Z_APPROACH_OFFSET = 0.04 # Offset for approaching objects safely from above (4cm)
FIXED_ORIENTATION = {"x": 1.0, "y": 0.0, "z": 0.0, "w": 0.0}
Z_INCREMENT = 0.001     # Fine increment for Z_ACTIVE adjustments

# --- Pencil Holder & Color Configuration ---
# ** CRITICAL: This is the height for picking up and placing pencils. **
PENCIL_PICKUP_Z = 0.125

PENCIL_BASE_X = 0.326
PENCIL_BASE_Y_LEFT = -0.32
PENCIL_BASE_Y_RIGHT = 0.32
PENCIL_X_OFFSET = 0.06

PENCIL_POSITIONS = {
    # Pencil Number: {position}
    1:  {"x": PENCIL_BASE_X + 0 * PENCIL_X_OFFSET, "y": PENCIL_BASE_Y_LEFT,  "z": PENCIL_PICKUP_Z},
    2:  {"x": PENCIL_BASE_X + 1 * PENCIL_X_OFFSET, "y": PENCIL_BASE_Y_LEFT,  "z": PENCIL_PICKUP_Z},
    3:  {"x": PENCIL_BASE_X + 2 * PENCIL_X_OFFSET, "y": PENCIL_BASE_Y_LEFT,  "z": PENCIL_PICKUP_Z},
    4:  {"x": PENCIL_BASE_X + 3 * PENCIL_X_OFFSET, "y": PENCIL_BASE_Y_LEFT,  "z": PENCIL_PICKUP_Z},
    5:  {"x": PENCIL_BASE_X + 4 * PENCIL_X_OFFSET, "y": PENCIL_BASE_Y_LEFT,  "z": PENCIL_PICKUP_Z},
    6:  {"x": PENCIL_BASE_X + 0 * PENCIL_X_OFFSET, "y": PENCIL_BASE_Y_RIGHT, "z": PENCIL_PICKUP_Z},
    7:  {"x": PENCIL_BASE_X + 1 * PENCIL_X_OFFSET, "y": PENCIL_BASE_Y_RIGHT, "z": PENCIL_PICKUP_Z},
    8:  {"x": PENCIL_BASE_X + 2 * PENCIL_X_OFFSET, "y": PENCIL_BASE_Y_RIGHT, "z": PENCIL_PICKUP_Z},
    9:  {"x": PENCIL_BASE_X + 3 * PENCIL_X_OFFSET, "y": PENCIL_BASE_Y_RIGHT, "z": PENCIL_PICKUP_Z},
    10: {"x": PENCIL_BASE_X + 4 * PENCIL_X_OFFSET, "y": PENCIL_BASE_Y_RIGHT, "z": PENCIL_PICKUP_Z},
}

COLOR_CODE_TO_PENCIL = {
    # HEX Color Code: Pencil Number
    "#16160F": 1, "#7A3D28": 2, "#D82929": 3, "#E86E09": 4, "#DBC416": 5,
    "#49C893": 6, "#02704D": 7, "#0D2875": 8, "#0295D5": 9, "#C0BDAE": 10,
}

# ==============================================================================
# --- HELPER FUNCTION FOR KEYBOARD INPUT ---
# ==============================================================================

def getch():
    """Gets a single character from standard input."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

# ==============================================================================
# --- MAIN ROBOT CONTROL CLASS ---
# ==============================================================================

class DrawingRobot:
    """A class to encapsulate robot state, communication, and actions."""

    def __init__(self):
        """Initializes the DrawingRobot instance."""
        rospy.init_node("drawing_robot_controller")

        # --- Workspace Boundaries ---
        self.x_min = WORKSPACE_CENTER_X - PAPER_WIDTH / 2
        self.x_max = WORKSPACE_CENTER_X + PAPER_WIDTH / 2
        self.y_min = WORKSPACE_CENTER_Y - PAPER_HEIGHT / 2
        self.y_max = WORKSPACE_CENTER_Y + PAPER_HEIGHT / 2
        
        # --- Robot State ---
        self.last_x = WORKSPACE_CENTER_X
        self.last_y = WORKSPACE_CENTER_Y
        self.last_z = Z_IDLE
        self.z_active = 0.125  # Drawing height, adjustable via keyboard
        self.processing_array = False
        self.current_pencil = None
        self.current_color_code = None
        
        # --- ROS Communication ---
        self.pose_publisher = rospy.Publisher("/demo/pose_final", PoseStamped, queue_size=10)
        self.gripper_move_client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
        self.gripper_grasp_client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)
        self.gripper_available = self._connect_to_gripper()

        self._log_startup_info()

    def _connect_to_gripper(self):
        """Waits for gripper action servers to become available."""
        rospy.loginfo("Connecting to gripper servers...")
        try:
            if self.gripper_move_client.wait_for_server(rospy.Duration(2.0)) and \
               self.gripper_grasp_client.wait_for_server(rospy.Duration(2.0)):
                rospy.loginfo("Gripper servers connected successfully.")
                return True
        except Exception as e:
            rospy.logwarn(f"Could not connect to gripper servers: {e}")
        rospy.logwarn("Proceeding without gripper control.")
        return False

    def _log_startup_info(self):
        """Logs the initial configuration."""
        rospy.loginfo("================= Drawing Robot Initialized =================")
        rospy.loginfo(f"Workspace: {SELECTED_PAPER} ({PAPER_WIDTH}m x {PAPER_HEIGHT}m)")
        rospy.loginfo(f"X Range: [{self.x_min:.3f}, {self.x_max:.3f}] | Y Range: [{self.y_min:.3f}, {self.y_max:.3f}]")
        rospy.loginfo(f"Initial Z (Active/Draw): {self.z_active:.3f}m | Z (Lift): {Z_LIFT:.3f}m")
        rospy.loginfo("Ready to receive TCP commands.")
        rospy.loginfo("=============================================================")

    def _publish_pose(self, x, y, z):
        """Publishes a target pose to the robot controller."""
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
        self.pose_publisher.publish(msg)
        # Update last known position
        self.last_x, self.last_y, self.last_z = x, y, z

    def _gradual_move(self, end_x, end_y, end_z, steps=10, delay=0.1):
        """Moves the robot smoothly from its last position to a new target."""
        start_x, start_y, start_z = self.last_x, self.last_y, self.last_z
        for i in range(1, steps + 1):
            progress = i / steps
            x = start_x + (end_x - start_x) * progress
            y = start_y + (end_y - start_y) * progress
            z = start_z + (end_z - start_z) * progress
            self._publish_pose(x, y, z)
            rospy.sleep(delay)

    def _map_coords(self, norm_x, norm_y):
        """Maps normalized (0-1) coordinates to the robot's workspace."""
        if FLIP_X: norm_x = 1.0 - norm_x
        if FLIP_Y: norm_y = 1.0 - norm_y
        
        # Swapped mapping: Canvas X -> Robot Y, Canvas Y -> Robot X
        real_x = self.x_min + norm_y * PAPER_WIDTH
        real_y = self.y_min + norm_x * PAPER_HEIGHT
        
        # Clamp to ensure it's within boundaries
        real_x = max(self.x_min, min(real_x, self.x_max))
        real_y = max(self.y_min, min(real_y, self.y_max))
        return real_x, real_y

    def return_pencil(self):
        """Returns the currently held pencil to its holder."""
        if self.current_pencil is None:
            return

        rospy.loginfo(f"Returning pencil #{self.current_pencil}...")
        pencil_pos = PENCIL_POSITIONS[self.current_pencil]
        approach_z = pencil_pos['z'] + Z_APPROACH_OFFSET

        # 1. Lift up from current position
        self._publish_pose(self.last_x, self.last_y, Z_LIFT)
        rospy.sleep(0.5)

        # 2. Move above the pencil holder
        self._gradual_move(pencil_pos['x'], pencil_pos['y'], approach_z, steps=20, delay=0.1)

        # 3. Lower to place the pencil
        self._gradual_move(pencil_pos['x'], pencil_pos['y'], pencil_pos['z'], steps=10, delay=0.1)

        # 4. Open gripper to release
        if self.gripper_available:
            self.gripper_move_client.send_goal_and_wait(MoveGoal(width=0.05, speed=0.1))
        rospy.sleep(1.0)

        # 5. Lift up away from the holder
        self._publish_pose(pencil_pos['x'], pencil_pos['y'], approach_z)
        rospy.sleep(0.5)

        self.current_pencil = None
        self.current_color_code = None
        rospy.loginfo("Pencil returned.")
        
    def pickup_pencil(self, pencil_number):
        """Executes the sequence to pick up a specific pencil."""
        if self.current_pencil == pencil_number:
            rospy.loginfo(f"Pencil #{pencil_number} is already held.")
            return
            
        if self.current_pencil is not None:
            self.return_pencil()

        rospy.loginfo(f"Picking up pencil #{pencil_number}...")
        pencil_pos = PENCIL_POSITIONS[pencil_number]
        approach_z = pencil_pos['z'] + Z_APPROACH_OFFSET
        
        # 1. Open gripper and move to a safe intermediate position
        if self.gripper_available:
             self.gripper_move_client.send_goal_and_wait(MoveGoal(width=0.05, speed=0.1))
        self._gradual_move(WORKSPACE_CENTER_X, WORKSPACE_CENTER_Y, Z_LIFT, steps=15, delay=0.1)

        # 2. Move above the target pencil
        self._gradual_move(pencil_pos['x'], pencil_pos['y'], approach_z, steps=20, delay=0.1)

        # 3. Lower to pickup height
        self._gradual_move(pencil_pos['x'], pencil_pos['y'], pencil_pos['z'], steps=10, delay=0.1)

        # 4. Grasp the pencil
        if self.gripper_available:
            grasp_goal = GraspGoal(width=0.024, speed=0.05, force=10.0)
            grasp_goal.epsilon.inner = grasp_goal.epsilon.outer = 0.005
            self.gripper_grasp_client.send_goal_and_wait(grasp_goal)
        rospy.sleep(1.0)
        
        # 5. Lift pencil up
        self._publish_pose(pencil_pos['x'], pencil_pos['y'], approach_z)
        rospy.sleep(0.5)
        
        # 6. Move back to the center of the workspace
        self._gradual_move(WORKSPACE_CENTER_X, WORKSPACE_CENTER_Y, Z_IDLE, steps=15, delay=0.1)

        self.current_pencil = pencil_number
        rospy.loginfo(f"Pencil #{pencil_number} pickup complete.")

    def process_coordinate_array(self, coordinates):
        """Moves the robot through a sequence of drawing coordinates."""
        if not coordinates:
            rospy.logwarn("Cannot process an empty coordinate array.")
            return

        self.processing_array = True
        rospy.loginfo(f"Processing array with {len(coordinates)} points...")

        # 1. Lift before moving to the start of the array
        self._publish_pose(self.last_x, self.last_y, Z_LIFT)
        rospy.sleep(0.5)

        # 2. Move to the first point at a safe height
        first_x, first_y = coordinates[0]
        self._publish_pose(first_x, first_y, Z_LIFT)
        rospy.sleep(0.3)
        
        # 3. Lower gradually to drawing height
        self._gradual_move(first_x, first_y, self.z_active, steps=5, delay=0.2)

        # 4. Execute the drawing path
        for x, y in coordinates:
            self._publish_pose(x, y, self.z_active)
            rospy.sleep(0.05) # Small delay for smoother movement

        # 5. Lift up after finishing the array
        self._publish_pose(self.last_x, self.last_y, Z_LIFT)
        rospy.sleep(0.3)
        
        rospy.loginfo("Array processing complete.")
        self.processing_array = False

    def handle_incoming_line(self, line):
        """Parses a line of text, changes pencil if needed, and processes coordinates."""
        parts = line.strip().split()
        if not parts:
            return

        color_code = None
        coords_text = line

        # Check if the first part is a color code
        if parts[0].startswith('#') and len(parts[0]) == 7:
            color_code = parts[0].upper()
            coords_text = ' '.join(parts[1:])
            rospy.loginfo(f"Received data with color: {color_code}")
        
        # --- Handle Pencil Change ---
        if color_code and color_code != self.current_color_code:
            pencil_to_get = COLOR_CODE_TO_PENCIL.get(color_code)
            if pencil_to_get:
                self.pickup_pencil(pencil_to_get)
                self.current_color_code = color_code
            else:
                rospy.logwarn(f"Color {color_code} not mapped to a pencil. Skipping.")
                return
        elif not self.current_pencil:
            rospy.logwarn("No color specified and no pencil held. Cannot draw.")
            return

        # --- Parse Coordinates ---
        coordinates = []
        for pair in coords_text.strip().split():
            try:
                x_str, y_str = pair.split(',', 1)
                # Normalize based on a 1400x1000 canvas size
                norm_x = float(x_str) / 1400.0
                norm_y = float(y_str) / 1000.0
                if 0.0 <= norm_x <= 1.0 and 0.0 <= norm_y <= 1.0:
                    coordinates.append(self._map_coords(norm_x, norm_y))
            except ValueError:
                rospy.logwarn(f"Could not parse coordinate pair: '{pair}'")
        
        # --- Process Array ---
        while self.processing_array:
            rospy.loginfo("Waiting for previous array to finish...")
            rospy.sleep(0.2)
        
        self.process_coordinate_array(coordinates)

    def tcp_listener(self):
        """Listens for incoming TCP data and processes it line by line."""
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            try:
                sock.connect((SERVER_IP, SERVER_PORT))
                rospy.loginfo(f"Connected to TCP server at {SERVER_IP}:{SERVER_PORT}")
                sock.settimeout(1.0)
                buffer = b""
                while not rospy.is_shutdown():
                    try:
                        data = sock.recv(4096)
                        if not data:
                            rospy.logwarn("TCP connection closed by server.")
                            break
                        buffer += data
                        while b"\n" in buffer:
                            line_bytes, buffer = buffer.split(b"\n", 1)
                            self.handle_incoming_line(line_bytes.decode('utf-8'))
                    except socket.timeout:
                        continue # Normal behavior when no data is sent
            except Exception as e:
                rospy.logerr(f"TCP connection failed or error during receive: {e}")
        rospy.loginfo("TCP listener thread stopped.")


def keyboard_listener(robot):
    """A simplified listener for adjusting Z height and quitting."""
    MIN_Z = 0.09
    MAX_Z = 0.25
    rospy.loginfo("Keyboard listener started: Use +/- to adjust drawing height, 'q' to quit.")

    while not rospy.is_shutdown():
        char = getch()
        if char in ['+', '=']:
            robot.z_active = min(MAX_Z, robot.z_active + Z_INCREMENT)
            rospy.loginfo(f"Drawing height (Z_ACTIVE) set to: {robot.z_active:.4f}m")
        elif char in ['-', '_']:
            robot.z_active = max(MIN_Z, robot.z_active - Z_INCREMENT)
            rospy.loginfo(f"Drawing height (Z_ACTIVE) set to: {robot.z_active:.4f}m")
        elif char in ['q', 'Q']:
            rospy.loginfo("Shutdown requested from keyboard.")
            robot.return_pencil()
            rospy.signal_shutdown("User quit.")
            break

def main():
    """Main function to start the robot controller."""
    try:
        robot = DrawingRobot()
        
        # Start listeners in separate threads
        tcp_thread = threading.Thread(target=robot.tcp_listener, daemon=True)
        keyboard_thread = threading.Thread(target=keyboard_listener, args=(robot,), daemon=True)
        
        tcp_thread.start()
        keyboard_thread.start()
        
        rospy.spin() # Keep the main thread alive until ROS shuts down
        
    except rospy.ROSInterruptException:
        print("ROS node interrupted.")
    except Exception as e:
        rospy.logfatal(f"An unhandled error occurred in main: {e}")
    finally:
        rospy.loginfo("Shutting down the drawing robot controller.")

if __name__ == "__main__":
    main()