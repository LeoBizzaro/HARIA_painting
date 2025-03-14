#include <ros/ros.h>
#include <vector>
#include <std_msgs/Bool.h>
#include <franka_msgs/FrankaState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <Eigen/Core>
#include <Eigen/Dense>

// Global variables
int current_index;
bool can_publish_next = false;
int can_get_list = 1;
int rows = 6;  // Now 6 poses for the hexagon
int index_list_el = 0;

// Define the six poses forming a hexagon (centered at the second pose)
geometry_msgs::Pose pose_sequence[6] = {
    [] {
        geometry_msgs::Pose p;
        p.position.x = 0.35; p.position.y = 0.1; p.position.z = 0.4;  // Top-left
        p.orientation.x = 1.0; p.orientation.y = 0.0; p.orientation.z = 0.0; p.orientation.w = 0.0;
        return p;
    }(),
    [] {
        geometry_msgs::Pose p;
        p.position.x = 0.4; p.position.y = 0.1; p.position.z = 0.4;  // Center
        p.orientation.x = 1.0; p.orientation.y = 0.0; p.orientation.z = 0.0; p.orientation.w = 0.0;
        return p;
    }(),
    [] {
        geometry_msgs::Pose p;
        p.position.x = 0.45; p.position.y = 0.1; p.position.z = 0.4;  // Top-right
        p.orientation.x = 1.0; p.orientation.y = 0.0; p.orientation.z = 0.0; p.orientation.w = 0.0;
        return p;
    }(),
    [] {
        geometry_msgs::Pose p;
        p.position.x = 0.45; p.position.y = 0.05; p.position.z = 0.4;  // Bottom-right
        p.orientation.x = 1.0; p.orientation.y = 0.0; p.orientation.z = 0.0; p.orientation.w = 0.0;
        return p;
    }(),
    [] {
        geometry_msgs::Pose p;
        p.position.x = 0.4; p.position.y = 0.0; p.position.z = 0.4;  // Bottom-center
        p.orientation.x = 1.0; p.orientation.y = 0.0; p.orientation.z = 0.0; p.orientation.w = 0.0;
        return p;
    }(),
    [] {
        geometry_msgs::Pose p;
        p.position.x = 0.35; p.position.y = 0.05; p.position.z = 0.4;  // Bottom-left
        p.orientation.x = 1.0; p.orientation.y = 0.0; p.orientation.z = 0.0; p.orientation.w = 0.0;
        return p;
    }()
};

geometry_msgs::PoseStamped msg;

int index_list[6];

Eigen::Vector3d current_position;
Eigen::Affine3d transform_current;

void nextPoseCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (msg->data) {
        can_publish_next = true;  // Flag
        ROS_INFO_STREAM("Moving to pose index: " << current_index);
    }
}

void frankaStateMessageReceived(const franka_msgs::FrankaState& robot_state) {
    transform_current = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());
    Eigen::Vector3d position(transform_current.translation());

    current_position = position;

    if (can_get_list == 1) {
        can_get_list = 0;

        Eigen::Vector3d pose_final = {pose_sequence[0].position.x, pose_sequence[0].position.y, pose_sequence[0].position.z};
        Eigen::Vector3d distance = current_position - pose_final;

        double min_distance = distance.norm();
        double current_distance = min_distance;

        for (int i = 1; i < rows; i++) {
            Eigen::Vector3d pose_vector = {pose_sequence[i].position.x, pose_sequence[i].position.y, pose_sequence[i].position.z};
            pose_final = pose_vector;
            distance = current_position - pose_final;
            current_distance = distance.norm();

            if (current_distance < min_distance) {
                min_distance = current_distance;
                current_index = i;
            }
        }

        int i = current_index;
        int order_pose = 0;
        double done = 0;

        while (done == 0) {
            index_list[order_pose] = i;
            i++;
            order_pose++;
            if (i == rows) {
                i = 0;
            }
            if (i == current_index) {
                done = 1;
            }
        }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "demo_pose_final_publisher");
    ros::NodeHandle nh;

    ros::Publisher pub_pose_final = nh.advertise<geometry_msgs::PoseStamped>("demo/pose_final", 1000);
    ros::Subscriber sub_nextPose = nh.subscribe("/demo/nextPose", 1000, nextPoseCallback);
    ros::Subscriber sub_currentPose = nh.subscribe("franka_state_controller/franka_states", 1000, frankaStateMessageReceived);
    ros::Rate rate(10);

    while (ros::ok()) {
        ros::spinOnce(); // Process callbacks

        if (can_publish_next) {
            msg.pose = pose_sequence[current_index];
            msg.header.frame_id = "fr3_link0";
            pub_pose_final.publish(msg);
            current_index = (current_index + 1) % rows;
            index_list_el++;
            ROS_INFO_STREAM("Publishing pose index " << current_index);
            can_publish_next = false;  // Reset flag
        }

        rate.sleep();
    }

    return 0;
}
