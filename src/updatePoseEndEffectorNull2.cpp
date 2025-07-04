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
#include <deque>

class UpdatePoseEndEffector
{
private:
    ros::NodeHandle nh;
    ros::Subscriber franka_states;
    ros::Subscriber pose_final;
    ros::Publisher pub_vel;
    ros::Publisher pub_nextPose;
    
    geometry_msgs::PoseStamped last_pose_final_stamped;
    geometry_msgs::Pose last_pose_final;
    
    Eigen::Affine3d transform_current;
    Eigen::Affine3d transform_final;

    Eigen::Vector3d current_position;
    Eigen::Vector3d final_position;
    Eigen::Quaterniond final_orientation;
    Eigen::Quaterniond current_orientation;
    Eigen::VectorXd dq, dq_meas, dq_nullspace;
    Eigen::VectorXd dq_prev;  // Store previous velocity command for smoother transitions

    // Changed from std::array to Eigen vectors
    Eigen::VectorXd jacobian_array;
    Eigen::Vector3d final_position_array;
    Eigen::Vector4d final_orientation_array;

    // Target trajectory smoothing
    Eigen::Vector3d smoothed_target_position;
    Eigen::Quaterniond smoothed_target_orientation;
    Eigen::Vector3d target_velocity;  // Estimated target velocity
    ros::Time last_target_update_time;
    bool first_target_received;  // Flag to handle first position update specially
    
    // FIFO queue for target position filtering
    std::deque<Eigen::Vector3d> position_history;
    std::deque<Eigen::Quaterniond> orientation_history;
    int history_size;

    // Preferred joint configuration
    Eigen::VectorXd q_desired;

    double dist_to_switch;
    double speed_constant;
    double safety_factor;
    double nullspace_gain;  // Gain for null-space control
    
    // Motion control parameters
    double position_smoothing_factor;  // Smoothing factor for target positions
    double orientation_smoothing_factor;  // Smoothing factor for target orientations
    double velocity_tracking_gain;  // How strongly to track estimated target velocity
    double max_acceleration;  // Maximum allowed acceleration
    double max_jerk;          // Maximum allowed jerk
    bool target_is_moving;    // Flag to indicate if target appears to be moving
    double target_speed;      // Estimated speed of the target
    ros::Time last_control_time; // Timestamp of last control loop
    
    // Velocity and acceleration state
    Eigen::Vector3d last_velocity_command;
    Eigen::Vector3d last_angular_velocity_command;
    Eigen::Vector3d current_acceleration;
    Eigen::Vector3d current_angular_acceleration;

public:
    UpdatePoseEndEffector() : nh("~")
    {
        pose_final = nh.subscribe("/demo/pose_final", 1000, &UpdatePoseEndEffector::poseFinalMessageReceived, this);
        franka_states = nh.subscribe("/franka_state_controller/franka_states", 1000, &UpdatePoseEndEffector::frankaStateMessageReceived, this);
        pub_vel = nh.advertise<std_msgs::Float64MultiArray>("/fr_joint_velocity_controller/command", 10);
        pub_nextPose = nh.advertise<std_msgs::Bool>("/demo/nextPose", 10);
        
        // Load parameters from ROS parameter server with defaults
        nh.param<double>("speed_constant", speed_constant, 0.5);    // Moderate baseline speed
        nh.param<double>("safety_factor", safety_factor, 1.0);
        nh.param<double>("nullspace_gain", nullspace_gain, 0.05);   // Small default null-space gain
        nh.param<double>("dist_to_switch", dist_to_switch, 0.005);  // 5mm threshold for goal reaching
        
        // Initialize advanced motion control parameters
        nh.param<double>("position_smoothing_factor", position_smoothing_factor, 0.85);
        nh.param<double>("orientation_smoothing_factor", orientation_smoothing_factor, 0.95);  // Higher value for more stable orientation
        nh.param<double>("velocity_tracking_gain", velocity_tracking_gain, 0.3);
        nh.param<double>("max_acceleration", max_acceleration, 0.8);  // Reduced for safer motion
        nh.param<double>("max_jerk", max_jerk, 3.0);  // Reduced for smoother motion
        nh.param<int>("history_size", history_size, 5);
        
        // Initialize first-time flag
        first_target_received = false;
        
        // Initialize vectors
        dq.resize(7);
        dq_meas.resize(7);
        dq_nullspace.resize(7);
        dq_prev.resize(7);
        q_desired.resize(7);
        dq_meas.setZero();
        dq_nullspace.setZero();
        dq.setZero();
        dq_prev.setZero();
        
        // Initialize motion state
        last_velocity_command.setZero();
        last_angular_velocity_command.setZero();
        current_acceleration.setZero();
        current_angular_acceleration.setZero();
        target_velocity.setZero();
        target_is_moving = false;
        target_speed = 0.0;
        first_target_received = false;
        
        // Initialize desired joint configuration from the paste.txt data
        q_desired << -0.0001488695516780325, -0.7855145696900614, 2.026375304975403e-05, 
                     -2.3560702091153742, 1.0639204139906155e-05, 1.5712664404489747, 
                     0.7854065589477477;
        
        jacobian_array.resize(42);
        final_position_array.setZero();
        final_orientation_array.setZero();
        smoothed_target_position.setZero();
        smoothed_target_orientation = Eigen::Quaterniond(1, 0, 0, 0);
        
        // Initialize the last_pose_final with zeros
        last_pose_final.position.x = 0;
        last_pose_final.position.y = 0;
        last_pose_final.position.z = 0;
        last_pose_final.orientation.x = 0;
        last_pose_final.orientation.y = 0;
        last_pose_final.orientation.z = 0;
        last_pose_final.orientation.w = 1;
        
        last_control_time = ros::Time::now();
        last_target_update_time = ros::Time::now();
        
        ROS_INFO("UpdatePoseEndEffector initialization complete with smoothing parameters:");
        ROS_INFO(" - Position smoothing: %.2f", position_smoothing_factor);
        ROS_INFO(" - Orientation smoothing: %.2f", orientation_smoothing_factor);
        ROS_INFO(" - Velocity tracking gain: %.2f", velocity_tracking_gain);
        ROS_INFO(" - Target history size: %d", history_size);
    }

    void poseFinalMessageReceived(const geometry_msgs::PoseStamped& pose) {
        // Extract position and orientation from incoming message
        Eigen::Vector3d new_position(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
        Eigen::Quaterniond new_orientation(pose.pose.orientation.w, pose.pose.orientation.x, 
                                        pose.pose.orientation.y, pose.pose.orientation.z);
        new_orientation.normalize();
        
        // For initial pose, we need special handling
        if (!first_target_received) {
            ROS_INFO("Received first target position. Initializing motion controller");
            
            // For the first target, directly use the orientation without filtering
            smoothed_target_position = new_position;
            smoothed_target_orientation = new_orientation;
            
            // Initialize history with the first target
            for (int i = 0; i < history_size; i++) {
                position_history.push_back(new_position);
                orientation_history.push_back(new_orientation);
            }
            
            first_target_received = true;
            
            // Update final position and orientation for the motion controller
            final_position = new_position;
            final_orientation = new_orientation;
            
            // Also update the arrays used elsewhere in the code
            final_position_array = final_position;
            final_orientation_array << final_orientation.x(), final_orientation.y(), 
                                      final_orientation.z(), final_orientation.w();
            
            // Update last pose
            last_pose_final_stamped = pose;
            last_pose_final = last_pose_final_stamped.pose;
            
            // Update the transform for later use
            updateFinalPose();
            last_target_update_time = ros::Time::now();
            
            ROS_INFO("Initial target set: Position (%.3f, %.3f, %.3f), Orientation (%.3f, %.3f, %.3f, %.3f)",
                   new_position.x(), new_position.y(), new_position.z(),
                   new_orientation.w(), new_orientation.x(), new_orientation.y(), new_orientation.z());
                   
            return;
        }
        
        // For subsequent poses - normal processing:
        // Estimate target velocity based on time difference and position change
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_target_update_time).toSec();
        last_target_update_time = current_time;
        
        // Only calculate velocity if we have previous data and time difference is reasonable
        if (dt > 0.01 && dt < 2.0 && last_pose_final_stamped.header.stamp.toSec() > 0) {
            Eigen::Vector3d last_pos(last_pose_final.position.x, 
                                     last_pose_final.position.y, 
                                     last_pose_final.position.z);
            
            Eigen::Vector3d instant_velocity = (new_position - last_pos) / dt;
            
            // Apply low-pass filter to velocity estimate
            target_velocity = 0.2 * instant_velocity + 0.8 * target_velocity;
            target_speed = target_velocity.norm();
            
            // Determine if target is significantly moving (only position, not orientation)
            target_is_moving = (target_speed > 0.01);  // 1 cm/s threshold
            
            if (target_is_moving) {
                ROS_INFO_THROTTLE(3.0, "Target moving at %.3f m/s", target_speed);
            }
        }
        
        // Verify orientation is valid (not NaN)
        if (std::isnan(new_orientation.x()) || std::isnan(new_orientation.y()) || 
            std::isnan(new_orientation.z()) || std::isnan(new_orientation.w())) {
            ROS_WARN("Received invalid orientation quaternion (NaN values). Using previous orientation.");
            new_orientation = smoothed_target_orientation;
        }

        // Update the position history queue
        position_history.push_back(new_position);
        orientation_history.push_back(new_orientation);
        
        // Trim history to configured size
        while (position_history.size() > history_size) {
            position_history.pop_front();
            orientation_history.pop_front();
        }
        
        // Compute smoothed target position by averaging recent positions
        Eigen::Vector3d avg_position = Eigen::Vector3d::Zero();
        
        for (auto& pos : position_history) {
            avg_position += pos;
        }
        avg_position /= position_history.size();
        
        // For orientation, we need special handling to maintain vertical orientation
        // First check if all orientations are close (to avoid averaging opposite quaternions)
        Eigen::Quaterniond ref_quat = orientation_history.front();
        
        // Check if requested orientation is significantly different from current
        double orientation_diff = 1.0 - std::abs(ref_quat.dot(new_orientation));
        bool significant_orientation_change = (orientation_diff > 0.05);  // ~15 degrees threshold
        
        // Always use the latest orientation directly - this is important to maintain vertical orientation
        // But use strong filtering to prevent jerky changes
        
        // Apply smoothing for a more gradual transition
        // Position smoothing - always smooth positions
        double p_factor = target_is_moving ? 
                        position_smoothing_factor * 0.8 :  // Less smoothing when target is moving
                        position_smoothing_factor;
        
        // Much stronger orientation smoothing - changes very gradually
        // Almost no orientation smoothing if there's a significant requested change
        double o_factor = significant_orientation_change ? 
                        orientation_smoothing_factor * 0.6 :  // Less smoothing when orientation changes
                        orientation_smoothing_factor;
                        
        smoothed_target_position = p_factor * smoothed_target_position + (1.0 - p_factor) * avg_position;
        
        // For quaternions, use slerp for smooth interpolation with strong filtering
        smoothed_target_orientation = smoothed_target_orientation.slerp(
            1.0 - o_factor, 
            new_orientation
        );
        
        // Update final position and orientation for the motion controller
        final_position = smoothed_target_position;
        final_orientation = smoothed_target_orientation;
        
        // Also update the arrays used elsewhere in the code
        final_position_array = final_position;
        final_orientation_array << final_orientation.x(), final_orientation.y(), 
                                  final_orientation.z(), final_orientation.w();
        
        // Update last pose
        last_pose_final_stamped = pose;
        last_pose_final = last_pose_final_stamped.pose;
        
        // Update the transform for later use
        updateFinalPose();
        
        // Log significant orientation changes
        if (significant_orientation_change) {
            ROS_INFO("Significant orientation change detected: %.3f degrees", orientation_diff * 180.0);
        }
    }

    void updateFinalPose()
    {
        // Directly use the smoothed target position and orientation
        Eigen::Matrix3d R = final_orientation.toRotationMatrix();
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

        T.block<3,3>(0,0) = R;
        T.block<3,1>(0,3) = final_position;
        transform_final = Eigen::Affine3d(T);
    }

    void frankaStateMessageReceived(const franka_msgs::FrankaState& robot_state) {
        
        // Special handling for initial setup - only if we haven't received a target yet
        if (!first_target_received && 
            (last_pose_final.position.x < 0.001 && last_pose_final.position.y < 0.001 && last_pose_final.position.z < 0.001)){ 
            transform_current = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());
            Eigen::Vector3d position(transform_current.translation());
            Eigen::Quaterniond orientation(transform_current.rotation());

            // Initialize everything with current pose
            final_position_array = position;
            final_orientation_array << orientation.x(), orientation.y(), orientation.z(), orientation.w();
            final_position = position;
            final_orientation = orientation;
            smoothed_target_position = position;
            smoothed_target_orientation = orientation;
            
            // Add to history queues
            while (position_history.size() < history_size) {
                position_history.push_back(position);
                orientation_history.push_back(orientation);
            }
            
            updateFinalPose();
            
            // Don't produce motion commands before receiving first target
            ROS_INFO_THROTTLE(5.0, "Waiting for first target position...");
            return;  // Skip control calculations for initial setup
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
        double dist = distance.norm();
        
        if(dist <= dist_to_switch)
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

        // Compute control update with consideration for time step
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_control_time).toSec();
        last_control_time = current_time;
        
        // Skip if dt is too small or too large (might be first iteration or pause in execution)
        if (dt < 0.001 || dt > 0.1) {
            dt = 0.01;  // Use a reasonable default
        }

        // Compute the task space velocity (twist) with prediction
        Eigen::VectorXd twist = computeTwist(current_orientation, current_position, dt);

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
        // Use reduced nullspace gain when target is moving for better stability
        double effective_nullspace_gain = target_is_moving ? 
                                         nullspace_gain * 0.5 : 
                                         nullspace_gain;
                                         
        dq_nullspace = null_projector * effective_nullspace_gain * (q_desired - q_current);
        
        // Combine primary and secondary tasks
        dq += dq_nullspace;
        
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
            // Adaptive filtering based on target movement
            double filter_factor;
            
            if (target_is_moving) {
                // Less filtering when target is moving to allow better tracking
                filter_factor = 0.6;
                
                // Even less filtering when we're far from the target
                if (dist > 0.05) {
                    filter_factor = 0.4;
                }
            } else {
                // More filtering for stable targets
                filter_factor = 0.8;
                
                // Less filtering when we're far from the target
                if (dist > 0.05) {
                    filter_factor = 0.5;
                }
            }
            
            // Apply the adaptive filter for velocity smoothing
            dq = filter_factor * dq_prev + (1.0 - filter_factor) * speed_constant * dq;
            
            // Store for next iteration
            dq_prev = dq;
            
            // Publish velocity command
            std_msgs::Float64MultiArray msg;
            msg.data.resize(7);
            for(int i = 0; i < 7; i++) {
                msg.data[i] = dq(i);
            }
            pub_vel.publish(msg);
            
            // Debug info
            ROS_INFO_THROTTLE(2.0, "Distance: %.3f m, Target moving: %s, Speed: %.3f m/s, Filter: %.2f", 
                            dist, 
                            target_is_moving ? "YES" : "NO",
                            target_speed,
                            filter_factor);
        }
    }

    Eigen::VectorXd computeTwist(const Eigen::Quaterniond& orientation, 
                               const Eigen::Vector3d& position,
                               double dt) {
        // Compute position error
        Eigen::Vector3d position_error = final_position - position;
        double dist = position_error.norm();
        
        // Compute desired linear velocity with predictive component
        Eigen::Vector3d linear_velocity;
        
        if (dist > dist_to_switch) {
            // Base movement on position error
            linear_velocity = position_error;
            
            // Scale the velocity based on distance - similar to a P controller with more gentle profile
            if (dist < 0.03) {
                // Close to target - slow down gradually
                linear_velocity *= std::max(0.2, dist / 0.03);  // Minimum 20% speed for smoother approach
            }
            else if (dist > 0.1) {
                // Far from target - cap max speed
                linear_velocity *= (0.7 + 0.3 * std::min(1.0, dist / 0.2));
            }
            
            // If the target is moving, add a feedforward term based on estimated target velocity
            if (target_is_moving) {
                linear_velocity += velocity_tracking_gain * target_velocity;
            }
            
            // Apply rate limiting for smooth acceleration
            constrainAcceleration(linear_velocity, last_velocity_command, 
                               current_acceleration, max_acceleration, max_jerk, dt);
            
            // Store current command for next iteration
            last_velocity_command = linear_velocity;
        }
        else {
            linear_velocity.setZero();
            last_velocity_command.setZero();
            current_acceleration.setZero();
        }

        // Compute angular velocity from quaternion difference
        Eigen::Quaterniond q_rel = final_orientation * orientation.inverse();
        q_rel.normalize();
        
        double w = q_rel.w();
        if(w > 1.0) w = 1.0;
        if(w < -1.0) w = -1.0;
        double theta = 2.0 * std::acos(w);

        Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
        
        // Only generate angular velocity commands when angle difference is significant
        // AND we're not in the initial phase of controller execution
        if (theta > 0.015 && first_target_received) {  // ~1 degree threshold to avoid noise
            Eigen::Vector3d axis = q_rel.vec();
            double norm = axis.norm();
            if(norm > 1e-6) {
                axis /= norm;
            }
            
            // Very reduced angular velocity compared to before - gentler orientation control
            angular_velocity = 0.5 * theta * axis;  // Reduced by factor of 2
            
            // Much gentler angular velocity scaling
            if (theta < 0.1) {
                // Small angle - reduce speed for precision
                angular_velocity *= (0.3 * theta / 0.1);  // Reduced to 30% for small angles
            } else if (theta > 0.5) {
                // Larger angle - still use gentler scaling
                angular_velocity *= (0.6 + 0.4 * std::min(1.0, theta / 1.0));
            }
            
            // Apply stronger constraint limits for orientation
            constrainAcceleration(angular_velocity, last_angular_velocity_command, 
                               current_angular_acceleration, max_acceleration * 0.5, max_jerk * 0.5, dt);
            
            // Store current command for next iteration
            last_angular_velocity_command = angular_velocity;
        }
        else {
            angular_velocity.setZero();
            last_angular_velocity_command.setZero();
            current_angular_acceleration.setZero();
        }

        // Combine linear and angular velocities into a twist
        Eigen::VectorXd twist(6);
        twist.head(3) = linear_velocity;
        twist.tail(3) = angular_velocity;
        
        // Debug orientation difference
        if (theta > 0.035) {  // Log only if angle difference > 2 degrees
            ROS_INFO_THROTTLE(2.0, "Orientation difference: %.2f degrees", theta * 180.0 / M_PI);
        }
        
        return twist;
    }
    
    void constrainAcceleration(Eigen::Vector3d& desired_velocity,
                            const Eigen::Vector3d& previous_velocity,
                            Eigen::Vector3d& current_acceleration,
                            double max_accel, double max_jerk, double dt) {
        // Calculate requested acceleration
        Eigen::Vector3d requested_acceleration = (desired_velocity - previous_velocity) / dt;
        double accel_magnitude = requested_acceleration.norm();
        
        // Limit jerk (rate of change of acceleration)
        Eigen::Vector3d acceleration_change = requested_acceleration - current_acceleration;
        double jerk_magnitude = acceleration_change.norm() / dt;
        
        if (jerk_magnitude > max_jerk) {
            acceleration_change *= (max_jerk / jerk_magnitude);
            // Apply limited acceleration change
            current_acceleration += acceleration_change * dt;
        } else {
            // No jerk limiting needed, update acceleration directly
            current_acceleration = requested_acceleration;
        }
        
        // Limit acceleration magnitude
        accel_magnitude = current_acceleration.norm();
        if (accel_magnitude > max_accel) {
            current_acceleration *= (max_accel / accel_magnitude);
        }
        
        // Compute limited velocity based on constrained acceleration
        desired_velocity = previous_velocity + current_acceleration * dt;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "update_pose_end_effector");
    ROS_INFO("Starting updatePoseEndEffectorSmooth node");
    
    UpdatePoseEndEffector update;
    
    ros::Rate loop_rate(100);  // 100 Hz control rate
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}