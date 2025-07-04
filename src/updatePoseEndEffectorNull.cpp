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
    
    // Added these declarations to fix the error
    geometry_msgs::PoseStamped last_pose_final_stamped;
    geometry_msgs::Pose last_pose_final;
    
    Eigen::Affine3d transform_current;
    Eigen::Affine3d transform_final;

    Eigen::Vector3d current_position;
    Eigen::Vector3d final_position;
    Eigen::Quaterniond final_orientation;
    Eigen::Quaterniond current_orientation;
    Eigen::VectorXd dq, dq_meas, dq_nullspace;

    // Changed from std::array to Eigen vectors
    Eigen::VectorXd jacobian_array;
    Eigen::Vector3d final_position_array;
    Eigen::Vector4d final_orientation_array;

    // Preferred joint configuration
    Eigen::VectorXd q_desired;

    double dist_to_switch;
    double speed_constant;
    double safety_factor;
    double nullspace_gain;  // Gain for null-space control

public:
    UpdatePoseEndEffector()
    {
        pose_final = nh.subscribe("demo/pose_final", 1000, &UpdatePoseEndEffector::poseFinalMessageReceived, this);
        franka_states = nh.subscribe("franka_state_controller/franka_states", 1000, &UpdatePoseEndEffector::frankaStateMessageReceived, this);
        pub_vel = nh.advertise<std_msgs::Float64MultiArray>("fr_joint_velocity_controller/command", 10);
        pub_nextPose = nh.advertise<std_msgs::Bool>("demo/nextPose", 10);
        
        // Initialize parameters with default values
        speed_constant = 0.4;  // Increased from 0.4 for faster movement
        safety_factor = 1.0;
        nullspace_gain = 0.05;  // Small default null-space gain (conservative)
        dist_to_switch = 0.005;
        
        // Initialize vectors
        dq.resize(7);
        dq_meas.resize(7);
        dq_nullspace.resize(7);
        q_desired.resize(7);
        dq_meas.setZero();
        dq_nullspace.setZero();
        dq.setZero();
        
        // Initialize desired joint configuration from the paste.txt data
        q_desired << -0.0001488695516780325, -0.7855145696900614, 2.026375304975403e-05, 
                     -2.3560702091153742, 1.0639204139906155e-05, 1.5712664404489747, 
                     0.7854065589477477;
        
        jacobian_array.resize(42);
        final_position_array.setZero();
        final_orientation_array.setZero();
        
        // Initialize the last_pose_final with zeros
        last_pose_final.position.x = 0;
        last_pose_final.position.y = 0;
        last_pose_final.position.z = 0;
        last_pose_final.orientation.x = 0;
        last_pose_final.orientation.y = 0;
        last_pose_final.orientation.z = 0;
        last_pose_final.orientation.w = 1;
        
        ROS_INFO("UpdatePoseEndEffector initialization complete");
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
        
        if ((last_pose_final.position.x < 0.001 && last_pose_final.position.y < 0.001 && last_pose_final.position.z < 0.001)){ 
            transform_current = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());
            Eigen::Vector3d position(transform_current.translation());
            Eigen::Quaterniond orientation(transform_current.rotation());

            final_position_array << position.x(), position.y(), position.z();
            final_orientation_array << orientation.x(), orientation.y(), orientation.z(), orientation.w();
            updateFinalPose();
        } 

        // Get current robot state
        transform_current = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());
        Eigen::Vector3d position(transform_current.translation());
        Eigen::Quaterniond orientation(transform_current.rotation());
        
        // Map Jacobian from the ROS message
        Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(robot_state.ZeroJacobian.data());
        
        // Get current joint positions
        Eigen::VectorXd q_current(7);
        for(int i = 0; i < 7; i++) {
            q_current(i) = robot_state.q[i];
        }
        
        // Get current joint velocities
        for(int i = 0; i < 7; i++) {
            dq_meas(i) = robot_state.dq[i];
        }
        
        current_position = position;
        current_orientation = orientation;

        // Check if we need to publish a nextPose message
        Eigen::Vector3d distance = final_position - current_position;
        if(distance.norm() <= dist_to_switch)
        {
            std_msgs::Bool msg;
            msg.data = true;
            pub_nextPose.publish(msg);
        }
        else {
            std_msgs::Bool msg;
            msg.data = false;
            pub_nextPose.publish(msg);
        }

        // Compute the task space velocity (twist)
        Eigen::VectorXd twist = computeTwist(current_orientation, current_position);

        // Compute the primary task joint velocities 
        // Use damped least squares method for pseudo-inverse
        double lambda = 0.01;  // Damping factor
        Eigen::Matrix<double, 6, 6> JJt = jacobian * jacobian.transpose() + lambda * Eigen::Matrix<double, 6, 6>::Identity();
        Eigen::Matrix<double, 7, 6> jacobian_pinv = jacobian.transpose() * JJt.inverse();
        
        // Compute null space projector (I - J+ * J)
        Eigen::Matrix<double, 7, 7> null_projector = Eigen::Matrix<double, 7, 7>::Identity() - jacobian_pinv * jacobian;
        
        // Compute primary task: joint velocities to achieve desired Cartesian motion
        dq = jacobian_pinv * twist;
        
        // Compute secondary task: null-space motion to move toward desired joint configuration
        dq_nullspace = null_projector * nullspace_gain * (q_desired - q_current);
        
        // Combine primary and secondary tasks
        dq += dq_nullspace;
        
        // Apply speed scaling and filtering
        nh.param("speed_constant", speed_constant, 0.7);  // Default increased from 0.4
        nh.param("nullspace_gain", nullspace_gain, 0.05);
        nh.param("safety_factor", safety_factor, 1.0);
        
        // Check for invalid values
        bool invalid = false;
        for(int i = 0; i < 7; i++) {
            if(std::isnan(dq(i)) || std::isinf(dq(i))) {
                invalid = true;
                break;
            }
        }
        
        // Apply velocity limits
        double max_joint_vel = 2.0;  // rad/s, adjust based on robot specs
        double max_vel = dq.cwiseAbs().maxCoeff();
        if(max_vel > max_joint_vel * safety_factor) {
            dq *= (max_joint_vel * safety_factor / max_vel);
        }
        
        // Publish joint velocities
        if (pub_vel.getNumSubscribers() > 0 && !invalid) {
            // Apply filtering with measured joint velocities (less smoothing for more responsive movement)
            dq = 0.8 * speed_constant * dq + 0.2 * dq_meas;
            
            // Publish velocity command
            std_msgs::Float64MultiArray msg;
            msg.data.resize(7);
            for(int i = 0; i < 7; i++) {
                msg.data[i] = dq(i);
            }
            pub_vel.publish(msg);
            
            // Debug info
            ROS_INFO_THROTTLE(5.0, "Primary task norm: %f, Nullspace task norm: %f", 
                             (jacobian_pinv * twist).norm(), dq_nullspace.norm());
        }
    }

    Eigen::VectorXd computeTwist(const Eigen::Quaterniond& orientation, const Eigen::Vector3d& position) {
        Eigen::Vector3d linear_velocity = final_position - position;
        
        // Scale linear velocity if distance is greater than threshold
        double dist = linear_velocity.norm();
        if(dist > dist_to_switch) {
            // Apply velocity scaling for smoother motion - with increased responsiveness
            double scale_factor = std::min(1.2, 0.7 + 0.8 * tanh(2.5 * (dist - 0.05)));
            linear_velocity = scale_factor * linear_velocity;
        }
        else {
            linear_velocity.setZero();
        }

        // Compute angular velocity from quaternion difference
        Eigen::Quaterniond q_rel = final_orientation * orientation.inverse();
        q_rel.normalize();
        
        double w = q_rel.w();
        if(w > 1.0) w = 1.0;
        if(w < -1.0) w = -1.0;
        double theta = 2.0 * std::acos(w);

        Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
        if (theta > 1e-6) {
            Eigen::Vector3d axis = q_rel.vec();
            double norm = axis.norm();
            if(norm > 1e-6) {
                axis /= norm;
            }
            angular_velocity = theta * axis;
            
            // Apply scaling with increased responsiveness
            double ang_scale = std::min(1.2, 0.5 + 0.7 * tanh(2.5 * theta));
            angular_velocity = ang_scale * angular_velocity;
        }

        // Combine linear and angular velocities into a twist
        Eigen::VectorXd twist(6);
        twist.head(3) = linear_velocity;
        twist.tail(3) = angular_velocity;
        
        return twist;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "update_pose_end_effector");
    ROS_INFO("Starting updatePoseEndEffectorNull node");
    
    UpdatePoseEndEffector update;
    
    ros::Rate loop_rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}