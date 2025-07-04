#!/usr/bin/env python3
import rospy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

def process_feedback(feedback):
    """ Callback per aggiornare la posizione quando il marker si muove """
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
    if x_min <= x <= x_max and y_min <= y <= y_max:
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "world"  # Cambia se necessario

        pose_msg.pose.position.x = x  # Mantiene la posizione x
        pose_msg.pose.position.y = y  # Mantiene la posizione y
        pose_msg.pose.position.z = 0.12  # Fissa la posizione z a 0.12
        
        pose_msg.pose.orientation.w = 0.0  # Imposta l'orientamento fisso
        pose_msg.pose.orientation.x = 1.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        
        pub.publish(pose_msg)  # Pubblica la nuova posizione solo se all'interno dell'area A3
        
        # Change color to green if within bounds
        update_marker_color(1.0, 0.0, 0.0)  # Red if within bounds
    else:
        # Change color to red if outside bounds
        update_marker_color(0.0, 0.0, 1.0)  # Blue if outside bounds

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

rospy.init_node("interactive_marker_node")

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
marker.pose.position.z = 0.12  # Modificato a 0.12m

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
bounds_marker.pose.position.z = 0.12

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

# Aggiungi il marker al server
server.insert(marker, process_feedback)
server.insert(bounds_marker)
server.applyChanges()

rospy.spin()