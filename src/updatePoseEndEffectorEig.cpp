#include <cstdlib>
#include <iterator>
#include <ros/ros.h>
#include <franka_msgs/FrankaState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <boost/array.hpp>
#include <iomanip>
#include <cmath>
#include <array>
#include <iostream>

class UpdatePoseEndEffector
{
private:
    ros::NodeHandle nh;
    ros::Subscriber franka_states;
    ros::Subscriber pose_final;
    ros::Publisher pub_vel;
    ros::Publisher pub_nextPose;
    
    Eigen::Affine3d transform_current;
    Eigen::Affine3d transform_final;

    Eigen::Vector3d current_position;
    Eigen::Vector3d final_position;
    Eigen::Quaterniond final_orientation;
    Eigen::Quaterniond current_orientation;
    Eigen::VectorXd dq,dq_meas;

    // Changed from std::array to Eigen vectors
    Eigen::VectorXd jacobian_array;
    Eigen::Vector3d final_position_array;
    Eigen::Vector4d final_orientation_array;

    franka::Robot *robot;
    franka::Model *model;
    franka::RobotState current_state;
    geometry_msgs::Pose last_pose_final;
    geometry_msgs::PoseStamped last_pose_final_stamped;



    double dist_to_switch;
    double speed_constant;
    double safety_factor;

public:
    UpdatePoseEndEffector()
    {
        pose_final = nh.subscribe("demo/pose_final", 1000, &UpdatePoseEndEffector::poseFinalMessageReceived,this);
        franka_states = nh.subscribe("franka_state_controller/franka_states",1000, &UpdatePoseEndEffector::frankaStateMessageReceived,this);
        pub_vel = nh.advertise<std_msgs::Float64MultiArray>("fr_joint_velocity_controller/command",10);
        pub_nextPose = nh.advertise<std_msgs::Bool>("demo/nextPose", 10);
        nh.setParam("speed_constant", 0.4);
        nh.setParam("safety_factor", 1);
        dist_to_switch = 0.005;
        dq.resize(7);
        dq_meas.resize(7);
        dq_meas << 0,0,0,0,0,0,0;
        jacobian_array.resize(42);
        final_position_array.setZero();
        final_orientation_array.setZero();
  }

    void poseFinalMessageReceived(const geometry_msgs::PoseStamped& pose) {
        //ROS_INFO("Got a pose!");

        last_pose_final_stamped = pose;
        last_pose_final = last_pose_final_stamped.pose;
        final_orientation_array << last_pose_final.orientation.x, last_pose_final.orientation.y, last_pose_final.orientation.z, last_pose_final.orientation.w;
        final_position_array << last_pose_final.position.x, last_pose_final.position.y, last_pose_final.position.z;

        updateFinalPose();

    }

    void updateFinalPose()
    {
        
        // Access vector data directly instead of using .data()
        Eigen::Vector3d position = final_position_array;
        Eigen::Quaterniond orientation(final_orientation_array(3), final_orientation_array(0), final_orientation_array(1), final_orientation_array(2));

        Eigen::Matrix3d R = orientation.toRotationMatrix();
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

        T.block<3,3>(0,0) = R;
        T.block<3,1>(0,3) = position;
        Eigen::Affine3d transform(T);

        transform_final = transform;
        final_position = position;
        final_orientation = orientation;
    }

    void frankaStateMessageReceived(const franka_msgs::FrankaState& robot_state) {

        /*
        if (last_pose_final.position.x < 0.0001 && last_pose_final.position.y < 0.0001 && last_pose_final.position.z < 0.0001) {
            final_position_array = {0.4,0,0.4};
            final_orientation_array = {1,0,0,0};
            updateFinalPose();
        } */

        
        if ((last_pose_final.position.x < 0.001 && last_pose_final.position.y < 0.001 && last_pose_final.position.z < 0.001)){ //ros::Time::now() -last_pose_final_stamped.header.stamp > ros::Duration(3.0)

            //ROS_INFO_STREAM_THROTTLE(1,"Initial pose! " << ros::Time::now() -last_pose_final_stamped.header.stamp);

            transform_current = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());
            Eigen::Vector3d position(transform_current.translation());
            Eigen::Quaterniond orientation(transform_current.rotation());

            final_position_array << position.x(), position.y(), position.z();
            final_orientation_array << orientation.x(), orientation.y(), orientation.z(), orientation.w();
            updateFinalPose();
        } 

        Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(robot_state.ZeroJacobian.data());
        //std::cout << jacobian << std::endl;
        transform_current = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());
        Eigen::Vector3d position(transform_current.translation());
        Eigen::Quaterniond orientation(transform_current.rotation());

        //for (int i=0;i<7;i++)         dq_meas[i]=robot_state.dq[i];
        dq_meas << robot_state.dq[0],robot_state.dq[1],robot_state.dq[2],robot_state.dq[3],
                    robot_state.dq[4],robot_state.dq[5],robot_state.dq[6];
        

        current_position = position;
        current_orientation = orientation;

        //std::cout << "The current position is:\n" << current_position << std::endl;
        //std::cout << "The current orientation is:\n" << current_orientation.coeffs() << std::endl;

        //std::cout << "The final position is:\n" << final_position << std::endl;
        //std::cout << "The final orientation is:\n" << final_orientation.coeffs() << std::endl;

        // Publishing on "/demo/nextPose" topic in case we are near the current final position
        Eigen::Vector3d distance = final_position - current_position;
        
        if(distance.norm() <= dist_to_switch)
        {
            std_msgs::Bool msg;
            msg.data = true;
            pub_nextPose.publish(msg);
            //std::cout << "The robot is close to the final pose" << std::endl;
        }
        else {
            std_msgs::Bool msg;
            msg.data = false;
            pub_nextPose.publish(msg);
        }

        Eigen::VectorXd twist = computeTwist(current_orientation, current_position);

        Eigen::ColPivHouseholderQR<Eigen::Matrix<double, 6, 7>> dec(jacobian);

        dq = dec.solve(twist);

        // Safety check
        //if(dq.maxCoeff() > 0.1*safety_factor) dq = dq*(0.1*safety_factor);

        bool invalid=(dq.array().isNaN()).any();


        //std::cout << "The twist is:\n" <<  invalid  << " subs: " << pub_vel.getNumSubscribers() << "\n" << twist << "\n##\n" << jacobian << std::endl;
        //nh.setParam("speed_constant", speed_constant);
        if (pub_vel.getNumSubscribers() > 0 && !invalid) {
            //nh.getParam("/speed_constant", );
            nh.param("speed_constant", speed_constant, 1.0);

            dq = 0.8*speed_constant*dq + 0.2*dq_meas; //exp filter
            //dq = speed_constant*dq;
            
            //std::cout << "The solution is:\n" << dq << std::endl;
            std_msgs::Float64MultiArray msg;
            msg.data.assign(dq.data(),dq.data()+dq.size());
            pub_vel.publish(msg);
        }
 
    }

    Eigen::VectorXd computeTwist(const Eigen::Quaterniond orientation, const Eigen::Vector3d position) {

        Eigen::Vector3d linear_velocity = final_position - position;
        
        // To have constant linear velocity
        Eigen::Vector3d dist_vect = Eigen::Vector3d::Constant(dist_to_switch);
        if(linear_velocity.norm() >= dist_to_switch)
        {
            //linear_velocity = linear_velocity/linear_velocity.norm();
        }
        else linear_velocity.setConstant(0);

        Eigen::Quaterniond q_rel = final_orientation*orientation.inverse();
        q_rel.normalize();
        
        double theta = 2 * std::acos(q_rel.w());

        Eigen::Vector3d axis = q_rel.vec();
        if (theta > 1e-6) {
            axis.normalize();
        } else {
            axis = Eigen::Vector3d::Zero();
        }

        Eigen::Vector3d angular_velocity = theta*axis;

        //std::cout << "The linear velocity is:" << linear_velocity << std::endl;
        //std::cout << "The angular velocity is:" << angular_velocity << std::endl;

        Eigen::VectorXd twist(6);
        twist << linear_velocity, angular_velocity;

        return twist;

    }

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "update_pose_end_effector");
    UpdatePoseEndEffector update;
    ros::Rate loop_rate(100);
    while (ros::ok()) {
        //update.variationPose();
        ros::spin();
        loop_rate.sleep();
    }
    return 0;
}