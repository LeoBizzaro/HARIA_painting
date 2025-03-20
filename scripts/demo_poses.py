#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped,Pose
from std_msgs.msg import Bool
import copy
import math


def callback1(msg):
    done1=msg.data
def callback2(msg):
    done2=msg.data


poses=[]
poses2=[]

done1=True
done2=True

rospy.init_node('pose_publisher', anonymous=True)


    
# Create a publisher that will publish to the /pose topic
pub = rospy.Publisher('/robot1/demo/pose_final', PoseStamped, queue_size=10)
pub2 = rospy.Publisher('/robot2/demo/pose_final', PoseStamped, queue_size=10)

sub1=rospy.Subscriber('/robot1/demo/nextPose', Bool, callback1)
sub2=rospy.Subscriber('/robot2/demo/nextPose', Bool, callback2)


def createPose(pos,ori):
    p=PoseStamped()
    p.header.frame_id='fr_link_0'
    p.pose.position.x = pos[0]; p.pose.position.y =pos[1]; p.pose.position.z = pos[2];  # Top-left
    p.pose.orientation.x = ori[0]; p.pose.orientation.y = ori[1]; p.pose.orientation.z = ori[2]; p.pose.orientation.w = ori[3];
    return p
'''
pose=PoseStamped()
p=Pose()
p.position.x = 0.35; p.position.y = 0.1; p.position.z = 0.4;  # Top-left
p.orientation.x = 1.0; p.orientation.y = 0.0; p.orientation.z = 0.0; p.orientation.w = 0.0;
pose.pose=p
poses.append(copy.deepcopy(pose))

p.position.x = 0.4; p.position.y = 0.1; p.position.z = 0.4;  #Center
p.orientation.x = 1.0; p.orientation.y = 0.0; p.orientation.z = 0.0; p.orientation.w = 0.0;
pose.pose=p
poses.append(copy.deepcopy(pose))

p.position.x = 0.45; p.position.y = 0.1; p.position.z = 0.4;  # Top-right
p.orientation.x = 1.0; p.orientation.y = 0.0; p.orientation.z = 0.0; p.orientation.w = 0.0;
pose.pose=p
poses.append(copy.deepcopy(pose))

p.position.x = 0.45; p.position.y = 0.05; p.position.z = 0.4;  # Bottom-right
p.orientation.x = 1.0; p.orientation.y = 0.0; p.orientation.z = 0.0; p.orientation.w = 0.0;
pose.pose=p
poses.append(copy.deepcopy(pose))


p.position.x = 0.4; p.position.y = 0.0; p.position.z = 0.4;  # Bottom-center
p.orientation.x = 1.0; p.orientation.y = 0.0; p.orientation.z = 0.0; p.orientation.w = 0.0;
pose.pose=p
poses.append(copy.deepcopy(pose))

p.position.x = 0.35; p.position.y = 0.05; p.position.z = 0.4;  #Bottom-left
p.orientation.x = 1.0; p.orientation.y = 0.0; p.orientation.z = 0.0; p.orientation.w = 0.0;
pose.pose=p
poses.append(copy.deepcopy(pose))


poses2.append(createPose([0.4,-0.1,0.4],[1,0,0,0]))
poses2.append(createPose([0.5,-0.2,0.4],[1,0,0,0]))
poses2.append(createPose([0.4,0.1,0.4],[1,0,0,0]))
'''

for i in range(20):
    t=i*2*math.pi/20
    poses.append(createPose([0.4+0.1*math.cos(t),0.1*math.sin(t),0.4],[1,0,0,0]))
    poses2.append(createPose([0.4+0.1*math.cos(t),-0.1*math.sin(t),0.4],[1,0,0,0]))



print(poses)
print(poses2)

r=rospy.Rate(1)

pub.publish(poses[0])
pub2.publish(poses2[0])
input("wait...")
        



while not rospy.is_shutdown():
    for i in range(len(poses)):

        while not (done1 and done2):
            r.sleep()

        print(poses[i%len(poses)])
        print(poses2[i%len(poses2)])



        poses[i%len(poses)].header.stamp=rospy.Time.now()
        poses2[i%len(poses2)].header.stamp=rospy.Time.now()

        pub.publish(poses[i%len(poses)])
        pub2.publish(poses2[i%len(poses2)])
        r.sleep()

        

