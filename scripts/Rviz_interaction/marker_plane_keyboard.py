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
import time
from franka_gripper.msg import MoveAction, MoveGoal, GraspAction, GraspGoal

# Global variables
gripper_closed = False
layer_timer = None
layer_transition_active = False

# Base and elevated z-heights (both shifted up by 1cm)
BASE_Z_HEIGHT = 0.12  # Original 0.12 + 0.01
ELEVATED_Z_HEIGHT = 0.22  # Original 0.22 + 0.01
LAYER_TRANSITION_DELAY = 3.0  # 3 seconds delay before layer transition

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

def layer_transition_callback():
    """Function to be called after the timer expires to switch layers"""
    global gripper_closed, layer_transition_active
    
    rospy.loginfo("Timer expired - switching layers")
    
    # Update marker and bounds positions
    update_marker_layer(gripper_closed)
    update_bounds_layer(gripper_closed)
    update_layer_indicator(gripper_closed)
    
    layer_transition_active = False

def schedule_layer_transition():
    """Schedule a layer transition after a delay"""
    global layer_timer, layer_transition_active
    
    if layer_transition_active:
        # Cancel the previous timer if it's still running
        if layer_timer is not None:
            layer_timer.cancel()
    
    # Start a new timer
    layer_transition_active = True
    
    # Using Timer from threading module
    layer_timer = threading.Timer(LAYER_TRANSITION_DELAY, layer_transition_callback)
    layer_timer.daemon = True
    layer_timer.start()
    
    rospy.loginfo(f"Layer transition scheduled in {LAYER_TRANSITION_DELAY} seconds")

def keyboard_listener():
    """Thread function to listen for keyboard inputs"""
    global gripper_closed
    
    rospy.loginfo("Keyboard control active: Press 'O' to open gripper, 'C' to close gripper, 'Q' to quit")
    
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
        
        if c == 'o' or c == 'O':
            rospy.loginfo("Opening gripper using MoveAction")
            goal = MoveGoal()
            goal.width = 0.08  # Opening width in meters
            goal.speed = 0.1   # Speed in m/s
            move_client.send_goal(goal)
            
            # Immediately update gripper state and schedule layer transition
            # without waiting for action result
            gripper_closed = False
            schedule_layer_transition()
                
        elif c == 'c' or c == 'C':
            rospy.loginfo("Closing gripper using GraspAction")
            goal = GraspGoal()
            goal.width = 0.02      # Target width when grasping
            goal.epsilon.inner = 0.005  # Tolerated deviation for grasp success
            goal.epsilon.outer = 0.005  # Tolerated deviation for grasp success
            goal.speed = 0.1       # Closing speed
            goal.force = 12        # Maximum grasp force in N
            grasp_client.send_goal(goal)
            
            # Immediately update gripper state and schedule layer transition
            # without waiting for action result
            gripper_closed = True
            schedule_layer_transition()
                
        elif c == 'q' or c == 'Q':
            rospy.loginfo("Quitting keyboard control")
            # Cancel any pending timer before exiting
            if layer_timer is not None:
                layer_timer.cancel()
            break

def update_marker_layer(is_closed):
    """Update the z-height of the interactive marker based on gripper state"""
    global server
    
    marker = server.get("target_pose")
    if marker:
        # Get current position
        current_x = marker.pose.position.x
        current_y = marker.pose.position.y
        
        # Set new z-height based on gripper state
        new_z = ELEVATED_Z_HEIGHT if is_closed else BASE_Z_HEIGHT
        
        # Update the marker's position
        marker.pose.position.z = new_z
        server.insert(marker)
        server.applyChanges()
        
        # Also update the robot pose via the publisher
        update_robot_pose(current_x, current_y, new_z)
        
        layer_name = "UPPER LAYER (10cm above)" if is_closed else "BASE LAYER"
        rospy.loginfo(f"Marker moved to {layer_name} at z={new_z}m")

def update_bounds_layer(is_closed):
    """Update the z-height of the bounds marker based on gripper state"""
    global server
    
    bounds = server.get("a3_bounds")
    if bounds:
        # Set new z-height based on gripper state
        new_z = ELEVATED_Z_HEIGHT if is_closed else BASE_Z_HEIGHT
        
        # Update the bounds marker's position
        bounds.pose.position.z = new_z
        server.insert(bounds)
        server.applyChanges()

def update_layer_indicator(is_closed):
    """Update the layer indicator text marker"""
    global server
    
    layer_marker = server.get("layer_indicator")
    if layer_marker:
        # Update the layer indicator position
        new_z = ELEVATED_Z_HEIGHT if is_closed else BASE_Z_HEIGHT
        layer_marker.pose.position.z = new_z
        
        # Update the text
        layer_text = "UPPER LAYER (GRIPPER CLOSED)" if is_closed else "BASE LAYER (GRIPPER OPEN)"
        for control in layer_marker.controls:
            for visual in control.markers:
                if visual.type == Marker.TEXT_VIEW_FACING:
                    visual.text = layer_text
        
        server.insert(layer_marker)
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

def process_feedback(feedback):
    """ Callback per aggiornare la posizione quando il marker si muove """
    global gripper_closed
    
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
    
    # Get current z-height based on gripper state
    current_z = ELEVATED_Z_HEIGHT if gripper_closed else BASE_Z_HEIGHT
    
    # Check if the position is within the A3 boundaries
    if x_min <= x <= x_max and y_min <= y <= y_max:
        # Update robot pose
        update_robot_pose(x, y, current_z)
        
        # Change color to red if within bounds
        update_marker_color(1.0, 0.0, 0.0)  # Red if within bounds
    else:
        # Change color to blue if outside bounds
        update_marker_color(0.0, 0.0, 1.0)  # Blue if outside bounds
        
        # Get the marker and keep it within bounds
        marker = server.get("target_pose")
        if marker:
            # Constrain x and y to stay within bounds
            constrained_x = min(max(x, x_min), x_max)
            constrained_y = min(max(y, y_min), y_max)
            
            # Update the marker position
            marker.pose.position.x = constrained_x
            marker.pose.position.y = constrained_y
            server.insert(marker)
            server.applyChanges()
            
            # Update robot pose with constrained position
            update_robot_pose(constrained_x, constrained_y, current_z)

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
    global layer_timer
    
    if layer_timer is not None:
        layer_timer.cancel()
    
    rospy.loginfo("Shutting down interactive marker node")

if __name__ == "__main__":
    rospy.init_node("interactive_marker_node")
    
    # Register shutdown hook
    rospy.on_shutdown(cleanup)

    # Publisher per inviare la pose al controllore del robot
    pub = rospy.Publisher("/demo/pose_final", PoseStamped, queue_size=10)

    # Server degli interactive markers
    server = InteractiveMarkerServer("interactive_marker_server")

    # Creazione del marker interattivo
    marker = InteractiveMarker()
    marker.header.frame_id = "world"  # Frame di riferimento
    marker.name = "target_pose"
    marker.description = "Drag within A3 paper area (297mm x 420mm)"
    marker.pose.position.x = 0.5  # Posizione iniziale (centro del foglio A3)
    marker.pose.position.y = 0.0
    marker.pose.position.z = BASE_Z_HEIGHT  # Base height (shifted up by 1cm)

    # Controllo del marker per movimento solo sul piano XY
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
    control.name = "move_xy"
    control.always_visible = True

    # Aggiungi una forma visibile direttamente a questo controllo
    visual = Marker()
    visual.type = Marker.CUBE  # Puoi cambiare in SPHERE o altro
    visual.scale.x = 0.025  # Dimensioni del cubo
    visual.scale.y = 0.025
    visual.scale.z = 0.025
    visual.color.r = 1.0  # Rosso iniziale
    visual.color.g = 0.0
    visual.color.b = 0.0
    visual.color.a = 1.0  # Opacità piena

    # Aggiungi il marker visivo al controllo principale
    control.markers.append(visual)
    marker.controls.append(control)

    # Aggiungi un marker per visualizzare i confini del foglio A3
    bounds_marker = InteractiveMarker()
    bounds_marker.header.frame_id = "world"
    bounds_marker.name = "a3_bounds"
    bounds_marker.description = "A3 Paper Boundaries"
    bounds_marker.pose.position.x = 0.5  # Centro dell'A3
    bounds_marker.pose.position.y = 0.0
    bounds_marker.pose.position.z = BASE_Z_HEIGHT  # Same as interactive marker

    bounds_control = InteractiveMarkerControl()
    bounds_control.always_visible = True

    # Marker per i confini del foglio A3
    bounds_visual = Marker()
    bounds_visual.type = Marker.CUBE
    bounds_visual.scale.x = 0.297  # larghezza A3 (297mm)
    bounds_visual.scale.y = 0.420  # altezza A3 (420mm)
    bounds_visual.scale.z = 0.001  # molto sottile
    bounds_visual.color.r = 0.8
    bounds_visual.color.g = 0.8
    bounds_visual.color.b = 0.8
    bounds_visual.color.a = 0.3  # Semi-trasparente

    bounds_control.markers.append(bounds_visual)
    bounds_marker.controls.append(bounds_control)

    # Add a text marker to indicate the current layer
    layer_marker = InteractiveMarker()
    layer_marker.header.frame_id = "world"
    layer_marker.name = "layer_indicator"
    layer_marker.description = "Current Layer"
    layer_marker.pose.position.x = 0.5
    layer_marker.pose.position.y = 0.25  # Just above the A3 sheet
    layer_marker.pose.position.z = BASE_Z_HEIGHT

    layer_control = InteractiveMarkerControl()
    layer_control.always_visible = True

    layer_visual = Marker()
    layer_visual.type = Marker.TEXT_VIEW_FACING
    layer_visual.text = "BASE LAYER (GRIPPER OPEN)"
    layer_visual.scale.z = 0.05  # Text size
    layer_visual.color.r = 1.0
    layer_visual.color.g = 1.0
    layer_visual.color.b = 1.0
    layer_visual.color.a = 1.0

    layer_control.markers.append(layer_visual)
    layer_marker.controls.append(layer_control)

    # Add a countdown timer visual marker
    timer_marker = InteractiveMarker()
    timer_marker.header.frame_id = "world"
    timer_marker.name = "timer_indicator"
    timer_marker.description = "Layer Transition Timer"
    timer_marker.pose.position.x = 0.5
    timer_marker.pose.position.y = 0.3  # Above the layer indicator
    timer_marker.pose.position.z = BASE_Z_HEIGHT

    timer_control = InteractiveMarkerControl()
    timer_control.always_visible = True

    timer_visual = Marker()
    timer_visual.type = Marker.TEXT_VIEW_FACING
    timer_visual.text = ""  # Initially empty, will be updated during transitions
    timer_visual.scale.z = 0.05  # Text size
    timer_visual.color.r = 1.0
    timer_visual.color.g = 0.5
    timer_visual.color.b = 0.0
    timer_visual.color.a = 1.0

    timer_control.markers.append(timer_visual)
    timer_marker.controls.append(timer_control)

    # Aggiungi i marker al server
    server.insert(marker, process_feedback)
    server.insert(bounds_marker)
    server.insert(layer_marker)
    server.insert(timer_marker)
    server.applyChanges()

    # Start keyboard listener thread
    keyboard_thread = threading.Thread(target=keyboard_listener)
    keyboard_thread.daemon = True  # Thread will close when main program exits
    keyboard_thread.start()

    rospy.loginfo("Interactive marker node started. Press 'O' to open gripper, 'C' to close gripper.")
    rospy.loginfo(f"Current layer: BASE LAYER at z={BASE_Z_HEIGHT}m")
    rospy.loginfo(f"Layer will transition after {LAYER_TRANSITION_DELAY} seconds delay")

    rospy.spin()