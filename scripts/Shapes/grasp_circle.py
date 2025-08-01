#!/usr/bin/env python3

import rospy
import math
import copy
import actionlib
import threading
import sys
import select
import termios
import tty
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Bool
from franka_gripper.msg import GraspAction, GraspGoal, GraspEpsilon

class PencilPickupNode:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('franka_pencil_pickup', anonymous=True)
       
        # Pose publisher for robot movement
        self.pose_pub = rospy.Publisher('/demo/pose_final', PoseStamped, queue_size=10)
       
        # Gripper action client - Changed to GraspAction
        self.grasp_client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)
        
        print("Waiting for gripper action server...")
        self.grasp_client.wait_for_server()
       
        # Next pose subscriber
        self.done = False
        self.next_pose_sub = rospy.Subscriber('/demo/nextPose', Bool, self.callback)
       
        # Pencil position
        self.pencil_pos = [0.493, 0, 0.21]
        
        # Home position
        self.home_pos = [0.4, 0, 0.4]
        self.home_ori = [1, 0, 0, 0]
       
        # Circle parameters
        self.center_x = 0.4
        self.center_y = 0
        self.center_z = 0.3
        self.radius = 0.3
        self.num_points = 20
       
        # Generate circle poses
        self.circle_poses = self.generate_circle_poses()
       
        # Rate for ROS operations
        self.rate = rospy.Rate(1)
        
        # Flag to control circular movement
        self.continue_circular_movement = True
   
    def callback(self, msg):
        """Callback for nextPose topic"""
        self.done = msg.data
   
    def create_pose(self, pos, ori=[1,0,0,0]):
        """Create a PoseStamped message"""
        p = PoseStamped()
        p.header.frame_id = 'fr_link_0'
        p.header.stamp = rospy.Time.now()
        p.pose.position.x = pos[0]
        p.pose.position.y = pos[1]
        p.pose.position.z = pos[2]
        p.pose.orientation.x = ori[0]
        p.pose.orientation.y = ori[1]
        p.pose.orientation.z = ori[2]
        p.pose.orientation.w = ori[3]
        return p
   
    def open_gripper(self):
        """Open gripper using Grasp action with high width"""
        goal = GraspGoal()
        goal.width = 0.08  # Open wide
        goal.epsilon.inner = 0.005
        goal.epsilon.outer = 0.005
        goal.speed = 0.1
        goal.force = 1.0  # Low force when opening
       
        print("Opening gripper")
        self.grasp_client.send_goal(goal)
        self.grasp_client.wait_for_result(rospy.Duration(5.0))
       
        if self.grasp_client.get_result():
            print("Gripper opened successfully")
        else:
            print("Failed to open gripper")
   
    def close_gripper_tightly(self):
        """Close gripper using Grasp action with force control"""
        goal = GraspGoal()
        goal.width = 0.004  # Target width for pencil
        goal.epsilon.inner = 0.005  # Tolerance for grasp
        goal.epsilon.outer = 0.005  # Tolerance for grasp
        goal.speed = 0.05  # Slower speed for more controlled grip
        goal.force = 5.0  # Apply specified force (in Newtons)
       
        print("Closing gripper with force control")
        self.grasp_client.send_goal(goal)
        self.grasp_client.wait_for_result(rospy.Duration(3.0))
       
        if self.grasp_client.get_result():
            print("Gripper closed with force control")
        else:
            print("Failed to close gripper with force control")
   
    def move_to_pose(self, pos, ori=[1,0,0,0]):
        """Move to a specific pose"""
        pose = self.create_pose(pos, ori)
        self.pose_pub.publish(pose)
        rospy.sleep(2)  # Wait for movement to complete
   
    def generate_circle_poses(self):
        """Generate poses for circular movement"""
        poses = []
        for i in range(self.num_points):
            t = i * 2 * math.pi / self.num_points
            poses.append(self.create_pose([
                self.center_x + self.radius * math.cos(t),
                self.center_y + self.radius * math.sin(t),
                self.center_z
            ]))
        return poses
   
    def is_data(self):
        """Check if there's input data"""
        return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])
    
    def move_to_home(self):
        """Move robot to home position"""
        print("Moving to home position...")
        self.move_to_pose(self.home_pos, self.home_ori)
        print("Robot is now at home position")

    def execute_circular_movement(self):
        """Execute continuous circular movement until interrupted"""
        print("Starting continuous circular movement. Press Enter to stop.")
        
        # Save terminal settings
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            # Change terminal settings to capture input without Enter
            tty.setcbreak(sys.stdin.fileno())
            
            while self.continue_circular_movement and not rospy.is_shutdown():
                # Iterate through circle poses
                for pose in self.circle_poses:
                    if not self.continue_circular_movement or rospy.is_shutdown():
                        break
                    
                    # Publish pose
                    pose.header.stamp = rospy.Time.now()
                    self.pose_pub.publish(pose)
                    
                    # Check for input
                    if self.is_data():
                        if sys.stdin.read(1) == '\n':
                            self.continue_circular_movement = False
                            print("\nCircular movement stopped.")
                            break
                    
                    # Small pause between poses
                    rospy.sleep(0.2)
                
                # Break outer loop if movement should stop
                if not self.continue_circular_movement:
                    break
        
        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
   
    def execute_pickup_and_circle(self):
        try:
            # Open gripper
            self.open_gripper()
           
            # Move to pencil position
            print("Moving to pencil position")
            self.move_to_pose(self.pencil_pos)
           
            # Wait for user input
            input("Press Enter to close gripper and pick up pencil...")
           
            # Close gripper with force control
            self.close_gripper_tightly()
           
            # Lift pencil slightly
            lift_pos = self.pencil_pos.copy()
            lift_pos[2] += 0.1  # Lift 10cm
            self.move_to_pose(lift_pos)
           
            # Wait for user to press enter
            input("Press Enter to start continuous circular movement... (Enter again to stop)")
           
            # Reset continue flag
            self.continue_circular_movement = True
           
            # Execute circular movement
            self.execute_circular_movement()
            
            # After circular movement stops, move to home position
            self.move_to_home()
            
            # Open gripper to release pencil
            self.open_gripper()
       
        except rospy.ROSInterruptException:
            rospy.loginfo("Interrupted")
        except KeyboardInterrupt:
            rospy.loginfo("Keyboard interrupt")
            # Try to move to home position even if interrupted
            try:
                self.move_to_home()
            except:
                rospy.loginfo("Could not move to home position after interruption")

def main():
    node = PencilPickupNode()
    node.execute_pickup_and_circle()

if __name__ == '__main__':
    main()