#!/usr/bin/env python3
import rospy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
import threading
import sys
import tty
import termios
import actionlib
from franka_gripper.msg import MoveAction, MoveGoal, GraspAction, GraspGoal

# Global variables
gripper_closed = False

# Height parameters
MIN_Z_HEIGHT = 0.12  # Lower limit
MAX_Z_HEIGHT = 0.50  # Upper limit
CURRENT_Z_HEIGHT = 0.12  # Starting height
Z_INCREMENT = 0.02  # 2cm increment for height adjustment

# Last valid position within bounds
last_valid_x = 0.5  # Initial position (center of A3 sheet)
last_valid_y = 0.0

def getch():
    """Get a single character from the user without requiring Enter"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def adjust_height(increment):
    """Adjust the height by the given increment, respecting limits"""
    global CURRENT_Z_HEIGHT
    
    new_height = CURRENT_Z_HEIGHT + increment
    if new_height < MIN_Z_HEIGHT:
        new_height = MIN_Z_HEIGHT
        rospy.loginfo(f"Reached minimum height of {MIN_Z_HEIGHT}m")
    elif new_height > MAX_Z_HEIGHT:
        new_height = MAX_Z_HEIGHT
        rospy.loginfo(f"Reached maximum height of {MAX_Z_HEIGHT}m")
    
    if new_height != CURRENT_Z_HEIGHT:
        CURRENT_Z_HEIGHT = new_height
        update_marker_height(new_height)
        rospy.loginfo(f"Height adjusted to: {CURRENT_Z_HEIGHT:.2f}m")
        
        # Update the robot's position using the last valid coordinates
        update_robot_pose(last_valid_x, last_valid_y, CURRENT_Z_HEIGHT)

def keyboard_listener():
    """Thread function to listen for keyboard inputs"""
    global gripper_closed
    
    rospy.loginfo("Keyboard control active:")
    rospy.loginfo("Press '+' to move up 2cm, '-' to move down 2cm")
    rospy.loginfo("Press 'O' to open gripper, 'C' to close gripper")
    rospy.loginfo("Press 'Q' to quit")
    
    # Action clients for gripper control
    move_client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
    grasp_client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)
    
    # Wait for the action servers to start
    rospy.loginfo("Waiting for gripper action servers...")
    move_client.wait_for_server()
    grasp_client.wait_for_server()
    rospy.loginfo("Gripper action servers connected!")
    
    while not rospy.is_shutdown():
        c = getch()
        
        if c == '+' or c == '=':  # Allow both '+' and '=' (unshifted '+')
            rospy.loginfo("Moving up 2cm")
            adjust_height(Z_INCREMENT)
                
        elif c == '-' or c == '_':  # Allow both '-' and '_' (shifted '-')
            rospy.loginfo("Moving down 2cm")
            adjust_height(-Z_INCREMENT)
            
        elif c == 'o' or c == 'O':
            rospy.loginfo("Opening gripper using MoveAction")
            goal = MoveGoal()
            goal.width = 0.08  # Opening width in meters
            goal.speed = 0.1   # Speed in m/s
            move_client.send_goal(goal)
            gripper_closed = False
                
        elif c == 'c' or c == 'C':
            rospy.loginfo("Closing gripper using GraspAction")
            goal = GraspGoal()
            goal.width = 0.02      # Target width when grasping
            goal.epsilon.inner = 0.005  # Tolerated deviation for grasp success
            goal.epsilon.outer = 0.005  # Tolerated deviation for grasp success
            goal.speed = 0.1       # Closing speed
            goal.force = 12        # Maximum grasp force in N
            grasp_client.send_goal(goal)
            gripper_closed = True
                
        elif c == 'q' or c == 'Q':
            rospy.loginfo("Quitting keyboard control")
            break

def update_marker_height(new_height):
    """Update the z-height of the interactive marker and bounds"""
    global server
    
    # Get the current marker position
    marker = server.get("target_pose")
    bounds = server.get("a3_bounds")
    
    if marker:
        # Keep X and Y position, just update Z
        marker.pose.position.z = new_height
        server.insert(marker)
    
    if bounds:
        # Update the bounds marker's z position
        bounds.pose.position.z = new_height
        server.insert(bounds)
    
    server.applyChanges()

def update_robot_pose(x, y, z):
    """Update the robot's pose with the new position"""
    pose_msg = PoseStamped()
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.header.frame_id = "world"
    
    pose_msg.pose.position.x = x
    pose_msg.pose.position.y = y
    pose_msg.pose.position.z = z
    
    pose_msg.pose.orientation.w = 0.0
    pose_msg.pose.orientation.x = 1.0
    pose_msg.pose.orientation.y = 0.0
    pose_msg.pose.orientation.z = 0.0
    
    pub.publish(pose_msg)

def is_within_bounds(x, y):
    """Check if the position is within the A3 boundaries"""
    # Define A3 paper dimensions in meters
    a3_width = 0.297  # 297mm in meters
    a3_height = 0.420  # 420mm in meters
    
    # Define the center of the A3 paper
    center_x = 0.5
    center_y = 0.0
    
    # Calculate the boundaries of the A3 paper
    x_min = center_x - a3_width/2
    x_max = center_x + a3_width/2
    y_min = center_y - a3_height/2
    y_max = center_y + a3_height/2
    
    return x_min <= x <= x_max and y_min <= y <= y_max

def process_feedback(feedback):
    """Callback to update position when marker moves"""
    global last_valid_x, last_valid_y
    
    # Get current position
    x = feedback.pose.position.x
    y = feedback.pose.position.y
    
    # Define A3 paper dimensions in meters
    a3_width = 0.297  # 297mm in meters
    a3_height = 0.420  # 420mm in meters
    
    # Define the center of the A3 paper
    center_x = 0.5
    center_y = 0.0
    
    # Calculate the boundaries of the A3 paper
    x_min = center_x - a3_width/2
    x_max = center_x + a3_width/2
    y_min = center_y - a3_height/2
    y_max = center_y + a3_height/2
    
    # Check if the position is within the A3 boundaries
    if is_within_bounds(x, y):
        # Update last valid position
        last_valid_x = x
        last_valid_y = y
        
        # Update robot pose with current position
        update_robot_pose(x, y, CURRENT_Z_HEIGHT)
        
        # Change color to red if within bounds
        update_marker_color(1.0, 0.0, 0.0)  # Red if within bounds
        
        rospy.logdebug(f"Marker is within bounds, robot following at ({x:.3f}, {y:.3f}, {CURRENT_Z_HEIGHT:.3f})")
    else:
        # Change color to blue if outside bounds
        update_marker_color(0.0, 0.0, 1.0)  # Blue if outside bounds
        
        # The robot stays at the last valid position
        rospy.logdebug(f"Marker outside bounds. Robot remains at ({last_valid_x:.3f}, {last_valid_y:.3f}, {CURRENT_Z_HEIGHT:.3f})")
        
        # Don't update the robot's position as we want it to stay at the last valid position
        # Only update the marker's color to indicate it's out of bounds

def update_marker_color(r, g, b):
    """Update the marker color"""
    marker = server.get("target_pose")
    if marker:
        # Update the color of the visual marker
        for control in marker.controls:
            for visual in control.markers:
                visual.color.r = r
                visual.color.g = g
                visual.color.b = b
        server.insert(marker)
        server.applyChanges()

def cleanup():
    """Function to be called when node is shutting down"""
    rospy.loginfo("Shutting down interactive marker node")

if __name__ == "__main__":
    rospy.init_node("interactive_marker_node")
    
    # Register shutdown hook
    rospy.on_shutdown(cleanup)

    # Publisher for sending the pose to the robot controller
    pub = rospy.Publisher("/demo/pose_final", PoseStamped, queue_size=10)

    # Interactive marker server
    server = InteractiveMarkerServer("interactive_marker_server")

    # Create interactive marker
    marker = InteractiveMarker()
    marker.header.frame_id = "world"
    marker.name = "target_pose"
    marker.description = ""  # Empty description to hide text
    marker.pose.position.x = 0.5  # Initial position (center of A3 sheet)
    marker.pose.position.y = 0.0
    marker.pose.position.z = CURRENT_Z_HEIGHT  # Initial height

    # Control for marker movement on XY plane
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0

    control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
    control.name = "move_xy"
    control.always_visible = True

    # Add a visible shape directly to this control
    visual = Marker()
    visual.type = Marker.CUBE
    visual.scale.x = 0.025  # Cube dimensions
    visual.scale.y = 0.025
    visual.scale.z = 0.025
    visual.color.r = 1.0  # Initial red color
    visual.color.g = 0.0
    visual.color.b = 0.0
    visual.color.a = 1.0  # Full opacity

    # Add visual marker to main control
    control.markers.append(visual)
    marker.controls.append(control)

    # Add marker to show A3 paper boundaries
    bounds_marker = InteractiveMarker()
    bounds_marker.header.frame_id = "world"
    bounds_marker.name = "a3_bounds"
    bounds_marker.description = ""  # Empty description to hide text
    bounds_marker.pose.position.x = 0.5  # A3 center
    bounds_marker.pose.position.y = 0.0
    bounds_marker.pose.position.z = CURRENT_Z_HEIGHT  # Same as interactive marker

    bounds_control = InteractiveMarkerControl()
    bounds_control.always_visible = True

    # Marker for A3 paper boundaries
    bounds_visual = Marker()
    bounds_visual.type = Marker.CUBE
    bounds_visual.scale.x = 0.297  # A3 width (297mm)
    bounds_visual.scale.y = 0.420  # A3 height (420mm)
    bounds_visual.scale.z = 0.001  # Very thin
    bounds_visual.color.r = 0.8
    bounds_visual.color.g = 0.8
    bounds_visual.color.b = 0.8
    bounds_visual.color.a = 0.3  # Semi-transparent

    bounds_control.markers.append(bounds_visual)
    bounds_marker.controls.append(bounds_control)

    # Add markers to server
    server.insert(marker, process_feedback)
    server.insert(bounds_marker)
    server.applyChanges()

    # Start keyboard listener thread
    keyboard_thread = threading.Thread(target=keyboard_listener)
    keyboard_thread.daemon = True  # Thread will close when main program exits
    keyboard_thread.start()

    rospy.loginfo("Interactive marker node started with continuous height adjustment")
    rospy.loginfo(f"Use '+' to move up 2cm, '-' to move down 2cm (limits: {MIN_Z_HEIGHT}m to {MAX_Z_HEIGHT}m)")
    rospy.loginfo("Press 'O' to open gripper, 'C' to close gripper, 'Q' to quit")
    rospy.loginfo("Robot will stay at boundary when marker is outside the A3 area")

    rospy.spin()