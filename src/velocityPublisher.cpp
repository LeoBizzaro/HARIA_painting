#include <ros/ros.h>
#include <array>

#include <std_msgs/Float64MultiArray.h>

void joinVelocityMessageReceived(const std_msgs::Float64MultiArray &msg) {
    ROS_INFO_STREAM("joint 1 =(" << msg.data[0]);
}

int main(int argc, char **argv) {
    // Initialize the ROS system and become a node.
    ros::init(argc, argv, "demo_velocities_publisher");
    ros::NodeHandle nh;

    //ros::Publisher pub_pose_final = nh.advertise<geometry_msgs::Pose>("demo/pose_final",1000); 
    ros::Subscriber joint_velocities = nh.subscribe("demo/joint_velocities",1000, &joinVelocityMessageReceived);
    ros::Rate rate(1000);

    //while (pub_pose_final.getNumSubscribers() == 0)
    //{
    //    ROS_INFO("Waiting for subscribers...");
    //    rate.sleep();
    //}

    // Let ROS take over.
    while (ros::ok()) {
        ros::spin();
        rate.sleep();
    }
    
}