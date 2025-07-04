#!/usr/bin/env python3
import rospy
from gazebo_msgs.msg import ModelStates
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
from geometry_msgs.msg import TransformStamped

class GazeboToRvizBridge:
    def __init__(self):
        self.marker_pub = rospy.Publisher('gazebo_objects', MarkerArray, queue_size=10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.last_update_time = rospy.Time.now()
        self.update_interval = rospy.Duration(0.1)  # Update at 10Hz to avoid TF warnings
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)
        self.marker_array = MarkerArray()
        
        # Keep track of the table position to place pencil correctly
        self.table_position = None

    def model_states_callback(self, data):
        # Only update if enough time has passed since the last update
        current_time = rospy.Time.now()
        if (current_time - self.last_update_time) < self.update_interval:
            return
        
        self.last_update_time = current_time
        
        # Clear previous markers
        if len(self.marker_array.markers) > 0:
            for marker in self.marker_array.markers:
                marker.action = Marker.DELETE
            self.marker_pub.publish(self.marker_array)
        
        # Create a new marker array
        self.marker_array = MarkerArray()
        marker_id = 0
        
        # First, find and process the table to get its position
        for i, model_name in enumerate(data.name):
            if model_name == 'table':
                self.table_position = data.pose[i].position
                
                # Table TF and marker
                table_tf = TransformStamped()
                table_tf.header.stamp = current_time
                table_tf.header.frame_id = "world"
                table_tf.child_frame_id = "table"
                table_tf.transform.translation.x = data.pose[i].position.x
                table_tf.transform.translation.y = data.pose[i].position.y
                table_tf.transform.translation.z = data.pose[i].position.z
                table_tf.transform.rotation = data.pose[i].orientation
                self.tf_broadcaster.sendTransform(table_tf)
                
                table_marker = Marker()
                table_marker.header.frame_id = "world"
                table_marker.header.stamp = current_time
                table_marker.ns = "gazebo_objects"
                table_marker.id = marker_id
                marker_id += 1
                table_marker.type = Marker.CUBE
                table_marker.action = Marker.ADD
                table_marker.pose = data.pose[i]
                table_marker.scale.x = 1.0
                table_marker.scale.y = 1.2
                table_marker.scale.z = 0.01
                table_marker.color.r = 0.7
                table_marker.color.g = 0.7
                table_marker.color.b = 0.7
                table_marker.color.a = 1.0
                self.marker_array.markers.append(table_marker)
                break
        
        # Process the pencil model if the table was found
        if self.table_position:
            for i, model_name in enumerate(data.name):
                if model_name == 'pencil':
                    # Pencil TF and marker - position it on top of the table
                    pencil_tf = TransformStamped()
                    pencil_tf.header.stamp = current_time
                    pencil_tf.header.frame_id = "world"
                    pencil_tf.child_frame_id = "pencil"
                    
                    # Position pencil on top of table
                    pencil_tf.transform.translation.x = data.pose[i].position.x
                    pencil_tf.transform.translation.y = data.pose[i].position.y
                    
                    # Table height (0.01) + half pencil height (0.1495/2) for proper placement
                    pencil_tf.transform.translation.z = self.table_position.z + 0.01 + (0.1495/2)
                    
                    pencil_tf.transform.rotation = data.pose[i].orientation
                    self.tf_broadcaster.sendTransform(pencil_tf)
                    
                    pencil_marker = Marker()
                    pencil_marker.header.frame_id = "world"
                    pencil_marker.header.stamp = current_time
                    pencil_marker.ns = "gazebo_objects"
                    pencil_marker.id = marker_id
                    marker_id += 1
                    pencil_marker.type = Marker.CUBE
                    pencil_marker.action = Marker.ADD
                    
                    # Copy the original pose first
                    pencil_marker.pose = data.pose[i]
                    
                    # Then override the z position to place on table
                    pencil_marker.pose.position.z = self.table_position.z + 0.01 + (0.1495/2)
                    
                    pencil_marker.scale.x = 0.027
                    pencil_marker.scale.y = 0.027
                    pencil_marker.scale.z = 0.1495
                    pencil_marker.color.r = 1.0
                    pencil_marker.color.g = 1.0
                    pencil_marker.color.b = 0.0
                    pencil_marker.color.a = 1.0
                    self.marker_array.markers.append(pencil_marker)
                    break
        
        # Publish all markers if we have any
        if len(self.marker_array.markers) > 0:
            self.marker_pub.publish(self.marker_array)
        else:
            rospy.logwarn("No table or pencil models found in Gazebo")

if __name__ == '__main__':
    rospy.init_node('gazebo_rviz_bridge')
    bridge = GazeboToRvizBridge()
    rospy.spin()