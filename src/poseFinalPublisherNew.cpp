#include <ros/ros.h>
#include <vector>
#include <std_msgs/Bool.h>
#include <franka_msgs/FrankaState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <Eigen/Core>
#include <Eigen/Dense>

// Global variables (the current index needs to be changed depending on which is the nearest position)
int current_index;
bool can_publish_next = false;
int can_get_list = 1;
int rows = 3; //3
int index_list_el = 0;


// Poses array (at the moment only 3 of them)
//std::vector<geometry_msgs::Pose> pose_sequence = {
geometry_msgs::Pose pose_sequence[3] = {
   [] {
        geometry_msgs::Pose p;
        p.position.x = 0.3; p.position.y = 0.0; p.position.z = 0.4;
        p.orientation.x = 1.0; p.orientation.y = 0.0; p.orientation.z = 0.0; p.orientation.w = 0.0;
        return p;
    }(),
    [] {
        geometry_msgs::Pose p;
        p.position.x = 0.4; p.position.y = 0.1; p.position.z = 0.4;
        p.orientation.x = 1.0; p.orientation.y = 0.0; p.orientation.z = 0.0; p.orientation.w = 0.0;
        return p;
    }(),
    [] {
        geometry_msgs::Pose p;
        p.position.x = 0.5; p.position.y = 0.0; p.position.z = 0.4;
        p.orientation.x = 1.0; p.orientation.y = 0.0; p.orientation.z = 0.0; p.orientation.w = 0.0;
        return p;
    }()

};

geometry_msgs::PoseStamped msg;

int index_list[3];

Eigen::Vector3d current_position;
Eigen::Affine3d transform_current;



void nextPoseCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (msg->data) {
        //current_index = (current_index + 1) % pose_sequence.size(); // To execute a cycle
        //current_index = (current_index + 1) % rows;
        can_publish_next = true;  // Flag
        ROS_INFO_STREAM("Moving to pose index: " << current_index);

    }
}

void frankaStateMessageReceived(const franka_msgs::FrankaState& robot_state) {

    transform_current = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());
    Eigen::Vector3d position(transform_current.translation());

    current_position = position;
    //std::cout << "The current position is:\n" << current_position << std::endl;

    if (can_get_list == 1) {
        
        can_get_list = 0;

        Eigen::Vector3d pose_final = {pose_sequence[0].position.x, pose_sequence[0].position.y, pose_sequence[0].position.z};

        Eigen::Vector3d distance = current_position - pose_final;

        double min_distance = distance.norm();
        double current_distance = min_distance;

        for (int i=1; i < rows; i++) {
            Eigen::Vector3d pose_vector = {pose_sequence[i].position.x, pose_sequence[i].position.y, pose_sequence[i].position.z};
            pose_final = pose_vector;
            //std::cout << "final position: " << pose_final << std::endl;
            //std::cout << "current position: " << current_position << std::endl;
            distance = current_position - pose_final;
            current_distance = distance.norm();
            //std::cout << "current distance: " << current_distance << std::endl;
            if (current_distance < min_distance) {
                min_distance = current_distance;
                current_index = i;
            }
            //std::cout << "min distance: " << min_distance << std::endl;
        }

        int i=current_index;
        int order_pose=0;
        double done=0;

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
    ros::Subscriber sub_nextPose = nh.subscribe("demo/nextPose", 1000, nextPoseCallback);
    ros::Subscriber sub_currentPose = nh.subscribe("franka_state_controller/franka_states",1000,frankaStateMessageReceived);
    ros::Rate rate(10);

    while (ros::ok()) {
        ros::spinOnce(); // For callback
        
        //std::cout << current_index << std::endl;

        //for (int i=0; i < rows; i++) {
        //    std::cout << index_list[i] << std::endl;
        //}



        if (can_publish_next) {
            msg.pose = pose_sequence[current_index];
            msg.header.frame_id = "fr3_link0";
            msg.header.stamp=ros::Time::now();
            pub_pose_final.publish(msg);
            current_index = (current_index + 1) % rows;
            index_list_el++;
            ROS_INFO_STREAM("Publishing pose index " << current_index);
            can_publish_next = false;  // Reset of the flag variable
        }

        //if (index_list_el == rows) {
        //    ros::shutdown();
        //}

        rate.sleep();
    }

    return 0;
}