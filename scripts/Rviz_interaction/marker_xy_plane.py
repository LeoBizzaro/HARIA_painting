#!/usr/bin/env python3
import rospy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

def process_feedback(feedback):
    """ Callback per aggiornare la posizione quando il marker si muove """
    pose_msg = PoseStamped()
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.header.frame_id = "world"  # Cambia se necessario

    pose_msg.pose.position.x = feedback.pose.position.x  # Mantiene la posizione x
    pose_msg.pose.position.y = feedback.pose.position.y  # Mantiene la posizione y
    pose_msg.pose.position.z = 0.12  # Fissa la posizione z a 0.12
    
    pose_msg.pose.orientation.w = 0.0  # Imposta l'orientamento fisso
    pose_msg.pose.orientation.x = 1.0
    pose_msg.pose.orientation.y = 0.0
    pose_msg.pose.orientation.z = 0.0
    
    pub.publish(pose_msg)  # Pubblica la nuova posizione

rospy.init_node("interactive_marker_node")

# Publisher per inviare la pose al controllore del robot
pub = rospy.Publisher("/demo/pose_final", PoseStamped, queue_size=10)

# Server degli interactive markers
server = InteractiveMarkerServer("interactive_marker_server")

# Creazione del marker interattivo
marker = InteractiveMarker()
marker.header.frame_id = "world"  # Frame di riferimento
marker.name = "target_pose"
marker.description = "Drag to move the target pose in XY plane"
marker.pose.position.x = 0.5  # Posizione iniziale
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
visual.color.r = 1.0  # Rosso
visual.color.g = 0.0
visual.color.b = 0.0
visual.color.a = 1.0  # Opacità piena

# Aggiungi il marker visivo al controllo principale
control.markers.append(visual)
marker.controls.append(control)

# Aggiungi il marker al server
server.insert(marker, process_feedback)
server.applyChanges()

rospy.spin()