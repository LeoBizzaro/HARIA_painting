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
    Eigen::VectorXd dq, dq_meas, dq_prev;

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
    double max_velocity;
    double filter_alpha;
    
    // Time-based variables for consistent control
    ros::Time last_update_time;
    double dt;  // Time delta between updates
    
    // New settings for improved responsiveness
    double min_filter_alpha;     // Minimum filtering when far from target
    double max_filter_alpha;     // Maximum filtering when near target
    double filter_transition_distance; // Distance at which filtering increases
    bool adaptive_filtering;     // Enable/disable adaptive filtering

public:
    UpdatePoseEndEffector()
    {
        pose_final = nh.subscribe("demo/pose_final", 1000, &UpdatePoseEndEffector::poseFinalMessageReceived,this);
        franka_states = nh.subscribe("franka_state_controller/franka_states",1000, &UpdatePoseEndEffector::frankaStateMessageReceived,this);
        pub_vel = nh.advertise<std_msgs::Float64MultiArray>("fr_joint_velocity_controller/command",10);
        pub_nextPose = nh.advertise<std_msgs::Bool>("demo/nextPose", 10);
        
        // Initialize parameters with optimized defaults for faster response
        nh.param("speed_constant", speed_constant, 0.7);  // Increased from 0.3 for faster movement
        nh.param("safety_factor", safety_factor, 1.0);
        nh.param("max_velocity", max_velocity, 0.8);      // Increased from 0.5 for faster movement
        nh.param("filter_alpha", filter_alpha, 0.7);      // Reduced from 0.85 for less filtering
        
        // New parameters for adaptive filtering
        nh.param("min_filter_alpha", min_filter_alpha, 0.3);  // Low filtering when far from target
        nh.param("max_filter_alpha", max_filter_alpha, 0.7);  // Higher filtering when close to target
        nh.param("filter_transition_distance", filter_transition_distance, 0.05); // 5cm transition zone
        nh.param("adaptive_filtering", adaptive_filtering, true); // Enable by default
        
        dist_to_switch = 0.005; // Distance threshold to consider position reached
        dq.resize(7);
        dq_meas.resize(7);
        dq_prev.resize(7);
        dq_meas.setZero();
        dq_prev.setZero();
        jacobian_array.resize(42);
        final_position_array.setZero();
        final_orientation_array.setZero();
        
        last_update_time = ros::Time::now();
        dt = 0.01;  // Initial estimate, will be updated in callback
  }

    void poseFinalMessageReceived(const geometry_msgs::PoseStamped& pose) {
        last_pose_final_stamped = pose;
        last_pose_final = last_pose_final_stamped.pose;
        final_orientation_array << last_pose_final.orientation.x, last_pose_final.orientation.y, last_pose_final.orientation.z, last_pose_final.orientation.w;
        final_position_array << last_pose_final.position.x, last_pose_final.position.y, last_pose_final.position.z;

        updateFinalPose();
    }

    void updateFinalPose()
    {
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
        // Update time delta for consistent control
        ros::Time now = ros::Time::now();
        dt = (now - last_update_time).toSec();
        last_update_time = now;
        
        // Ensure dt is reasonable to prevent instability
        if (dt < 0.001 || dt > 0.1) {
            dt = 0.01;  // Default to 10ms if time delta is unreasonable
        }
        
        if ((last_pose_final.position.x < 0.001 && last_pose_final.position.y < 0.001 && last_pose_final.position.z < 0.001)) {
            transform_current = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());
            Eigen::Vector3d position(transform_current.translation());
            Eigen::Quaterniond orientation(transform_current.rotation());

            final_position_array << position.x(), position.y(), position.z();
            final_orientation_array << orientation.x(), orientation.y(), orientation.z(), orientation.w();
            updateFinalPose();
        } 

        Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(robot_state.ZeroJacobian.data());
        transform_current = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());
        Eigen::Vector3d position(transform_current.translation());
        Eigen::Quaterniond orientation(transform_current.rotation());

        dq_meas << robot_state.dq[0], robot_state.dq[1], robot_state.dq[2], robot_state.dq[3],
                   robot_state.dq[4], robot_state.dq[5], robot_state.dq[6];
        
        current_position = position;
        current_orientation = orientation;

        // Calculate distance to target for adaptive filtering
        Eigen::Vector3d distance = final_position - current_position;
        double dist = distance.norm();
        
        // Publishing on "/demo/nextPose" topic in case we are near the current final position
        if(dist <= dist_to_switch) {
            std_msgs::Bool msg;
            msg.data = true;
            pub_nextPose.publish(msg);
        } else {
            std_msgs::Bool msg;
            msg.data = false;
            pub_nextPose.publish(msg);
        }

        // Compute twist with improved velocity scaling
        Eigen::VectorXd twist = computeTwist(current_orientation, current_position);

        // Use damped least squares for improved stability with reduced damping for faster response
        Eigen::MatrixXd J = jacobian;
        double lambda = 0.05;  // Reduced damping factor from 0.1 for faster response
        Eigen::MatrixXd JJT = J * J.transpose() + lambda * Eigen::MatrixXd::Identity(6, 6);
        Eigen::VectorXd v_dls = J.transpose() * JJT.ldlt().solve(twist);
        
        // Apply speed constant - load from parameter server each time to allow dynamic reconfigure
        nh.param("speed_constant", speed_constant, 0.7);
        v_dls = speed_constant * v_dls;

        // Check for NaN or infinite values
        bool invalid = (v_dls.array().isNaN() || v_dls.array().isInf()).any();

        if (pub_vel.getNumSubscribers() > 0 && !invalid) {
            // Get adaptive filter parameter
            nh.param("adaptive_filtering", adaptive_filtering, true);
            
            // Determine appropriate filter value based on distance to target
            double current_filter_alpha;
            
            if (adaptive_filtering) {
                nh.param("min_filter_alpha", min_filter_alpha, 0.3);
                nh.param("max_filter_alpha", max_filter_alpha, 0.5);
                nh.param("filter_transition_distance", filter_transition_distance, 0.03);
                
                if (dist > filter_transition_distance) {
                    // Far from target - minimal filtering for fast approach
                    current_filter_alpha = min_filter_alpha;
                } else if (dist < dist_to_switch) {
                    // Very close to target - maximum filtering for stability
                    current_filter_alpha = max_filter_alpha;
                } else {
                    // Transition zone - linearly interpolate filtering
                    double ratio = (dist - dist_to_switch) / (filter_transition_distance - dist_to_switch);
                    current_filter_alpha = max_filter_alpha + ratio * (min_filter_alpha - max_filter_alpha);
                }
            } else {
                // Use fixed filter value from parameter server
                nh.param("filter_alpha", filter_alpha, 0.5);
                current_filter_alpha = filter_alpha;
            }
            
            // Apply reduced filtering for faster response
            // Single-stage filtering with smaller alpha for more direct control
            dq = current_filter_alpha * dq_prev + (1.0 - current_filter_alpha) * v_dls;
            dq_prev = dq;  // Store for next iteration
            
            // Get max velocity from parameter server to allow dynamic adjustment
            nh.param("max_velocity", max_velocity, 0.8);
            
            // Apply velocity limits with safety margin
            for (int i = 0; i < 7; i++) {
                if (std::abs(dq(i)) > max_velocity) {
                    dq(i) = (dq(i) > 0) ? max_velocity : -max_velocity;
                }
            }
            
            // Publish velocity command
            std_msgs::Float64MultiArray msg;
            msg.data.assign(dq.data(), dq.data() + dq.size());
            pub_vel.publish(msg);
        }
    }

    Eigen::VectorXd computeTwist(const Eigen::Quaterniond orientation, const Eigen::Vector3d position) {
        Eigen::Vector3d linear_velocity = final_position - position;
        double dist = linear_velocity.norm();
        
        // Improved velocity scaling based on distance to target
        double vel_scale = 1.0;
        
        // Less velocity reduction to reach target faster
        if (dist < 0.03) {  // Reduced from 0.05 for more aggressive approach
            // Linear scaling for simpler, more direct control
            vel_scale = std::min(1.0, dist / 0.03);
        }
        
        // Apply distance-based scaling to prevent oscillations near target
        if (dist >= dist_to_switch) {
            linear_velocity = vel_scale * linear_velocity;
        } else {
            linear_velocity.setZero();
        }

        // Quaternion-based orientation control
        Eigen::Quaterniond q_rel = final_orientation * orientation.inverse();
        q_rel.normalize();
        
        // Compute rotation angle and axis
        double theta = 2.0 * std::acos(std::max(-1.0, std::min(1.0, q_rel.w())));
        
        Eigen::Vector3d axis;
        if (theta > 1e-6) {
            axis = q_rel.vec() / std::sin(theta/2.0);
            axis.normalize();
        } else {
            axis.setZero();
        }

        // More aggressive angular velocity for faster orientation changes
        Eigen::Vector3d angular_velocity = vel_scale * theta * axis;
        
        // Reduced damping on angular velocity for faster response
        if (angular_velocity.norm() < 0.03) {  // Reduced from 0.05
            angular_velocity *= angular_velocity.norm() / 0.03;
        }

        // Assemble the complete twist vector
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
        ros::spin();
        loop_rate.sleep();
    }
    return 0;
}