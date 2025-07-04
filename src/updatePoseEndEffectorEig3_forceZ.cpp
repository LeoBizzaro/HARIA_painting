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
#include <deque>
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

    Eigen::VectorXd jacobian_array;
    Eigen::Vector3d final_position_array;
    Eigen::Vector4d final_orientation_array;

    franka::Robot *robot;
    franka::Model *model;
    franka::RobotState current_state;
    geometry_msgs::Pose last_pose_final;
    geometry_msgs::PoseStamped last_pose_final_stamped;

    double dist_to_switch;
    double safety_factor;
    double max_velocity;
    
    // Time-based variables for consistent control
    ros::Time last_update_time;
    double dt;
    
    // VR Real-time following parameters
    bool vr_mode_enabled;              // Enable VR real-time mode
    double vr_following_gain;          // Proportional gain for VR following
    double vr_max_linear_vel;          // Maximum linear velocity for VR mode
    double vr_max_angular_vel;         // Maximum angular velocity for VR mode
    double vr_damping_factor;          // Light damping to prevent oscillations
    
    // Predictive following
    bool use_predictive_control;       // Enable velocity-based prediction
    Eigen::Vector3d previous_target_pos;
    Eigen::Vector3d target_velocity;   // Estimated target velocity
    ros::Time last_target_time;
    
    // Minimal filtering for real-time response
    double realtime_filter_alpha;      // Very light filtering (0.05-0.1)
    
    // Safety limits
    double emergency_stop_distance;    // Distance to trigger emergency stop
    double max_acceleration;           // Maximum joint acceleration
    Eigen::VectorXd previous_velocity; // For acceleration limiting
    
    // VR-specific parameters
    double vr_precision_threshold;     // Distance below which to use precision mode
    double precision_gain_factor;      // Reduced gain for precision movements
    
    // Velocity smoothing (minimal for real-time)
    std::deque<Eigen::VectorXd> velocity_buffer;
    int max_buffer_size;

    // Force monitoring variables
    double force_threshold_z;           // Configurable force threshold
    double current_force_z;             // Current force along z-axis
    double saturated_z_position;        // Z position when saturation occurs
    bool z_position_saturated;          // Flag to track saturation state

public:
    UpdatePoseEndEffector()
    {
        pose_final = nh.subscribe("demo/pose_final", 1000, &UpdatePoseEndEffector::poseFinalMessageReceived,this);
        franka_states = nh.subscribe("franka_state_controller/franka_states",1000, &UpdatePoseEndEffector::frankaStateMessageReceived,this);
        pub_vel = nh.advertise<std_msgs::Float64MultiArray>("fr_joint_velocity_controller/command",10);
        pub_nextPose = nh.advertise<std_msgs::Bool>("demo/nextPose", 10);
        
        // VR Real-time parameters - optimized for 60Hz VR input
        nh.param("vr_mode_enabled", vr_mode_enabled, true);
        nh.param("vr_following_gain", vr_following_gain, 3.0);           // Higher gain for responsiveness
        nh.param("vr_max_linear_vel", vr_max_linear_vel, 0.4);          // 80cm/s max linear velocity
        nh.param("vr_max_angular_vel", vr_max_angular_vel, 1.0);        // 2 rad/s max angular velocity
        nh.param("vr_damping_factor", vr_damping_factor, 0.02);         // Very light damping
        
        // Predictive control
        nh.param("use_predictive_control", use_predictive_control, true);
        
        // Minimal filtering for real-time response
        nh.param("realtime_filter_alpha", realtime_filter_alpha, 0.05);  // Minimal filtering
        
        // Safety parameters
        nh.param("safety_factor", safety_factor, 1.0);
        nh.param("max_velocity", max_velocity, 0.5);                     // Higher max velocity
        nh.param("emergency_stop_distance", emergency_stop_distance, 0.4); // 30cm emergency stop
        nh.param("max_acceleration", max_acceleration, 5.0);             // Joint acceleration limit
        
        // VR precision parameters
        nh.param("vr_precision_threshold", vr_precision_threshold, 0.01); // 1cm precision zone
        nh.param("precision_gain_factor", precision_gain_factor, 0.7);    // Reduced gain for precision
        
        // Initialization of force treshold
        nh.param("force_threshold_z", force_threshold_z, 2.0);  // Positive value (upward force)

        current_force_z = 0.0;
        saturated_z_position = 0.0;
        z_position_saturated = false;

        dist_to_switch = 0.002; // 2mm threshold for "reached" status
        dq.resize(7);
        dq_meas.resize(7);
        dq_prev.resize(7);
        previous_velocity.resize(7);
        dq_meas.setZero();
        dq_prev.setZero();
        previous_velocity.setZero();
        jacobian_array.resize(42);
        final_position_array.setZero();
        final_orientation_array.setZero();
        
        // Initialize predictive control variables
        previous_target_pos.setZero();
        target_velocity.setZero();
        last_target_time = ros::Time::now();
        
        // Initialize minimal velocity smoothing
        max_buffer_size = 3; // Very small buffer for minimal delay
        
        last_update_time = ros::Time::now();
        dt = 1.0/60.0;  // Assume 60Hz VR input
        
        ROS_INFO("VR Real-time Drawing Controller Initialized");
        ROS_INFO("VR Mode: %s, Following Gain: %.2f, Max Linear Vel: %.2f m/s", 
                 vr_mode_enabled ? "Enabled" : "Disabled", vr_following_gain, vr_max_linear_vel);
    }

    void poseFinalMessageReceived(const geometry_msgs::PoseStamped& pose) {
        last_pose_final_stamped = pose;
        last_pose_final = last_pose_final_stamped.pose;
        final_orientation_array << last_pose_final.orientation.x, last_pose_final.orientation.y, 
                                   last_pose_final.orientation.z, last_pose_final.orientation.w;
        final_position_array << last_pose_final.position.x, last_pose_final.position.y, last_pose_final.position.z;

        // Update target velocity estimation for predictive control
        if (use_predictive_control) {
            updateTargetVelocityEstimation();
        }

        updateFinalPose();
    }

    void updateTargetVelocityEstimation() {
        ros::Time now = ros::Time::now();
        double time_delta = (now - last_target_time).toSec();
        
        if (time_delta > 0.001 && time_delta < 0.1) { // Valid time delta
            //Eigen::Vector3d position_delta = final_position_array - previous_target_pos;
            Eigen::Vector3d position_delta = final_position_array - current_position;
            target_velocity = position_delta / time_delta;
            
            // Limit estimated velocity to reasonable values
            if (target_velocity.norm() > vr_max_linear_vel * 2.0) {
                target_velocity = target_velocity.normalized() * vr_max_linear_vel * 2.0;
            }
        }
        
        previous_target_pos = final_position_array;
        last_target_time = now;
    }

    void updateFinalPose()
    {
        Eigen::Vector3d position = final_position_array;
        Eigen::Quaterniond orientation(final_orientation_array(3), final_orientation_array(0), 
                                       final_orientation_array(1), final_orientation_array(2));

        Eigen::Matrix3d R = orientation.toRotationMatrix();
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

        T.block<3,3>(0,0) = R;
        T.block<3,1>(0,3) = position;
        Eigen::Affine3d transform(T);

        // Apply z-axis saturation if active
        if (z_position_saturated && position.z() < saturated_z_position) {
            position.z() = saturated_z_position;  // Clamp z to saturated position
        }

        transform_final = transform;
        final_position = position;
        final_orientation = orientation;
    }

    void frankaStateMessageReceived(const franka_msgs::FrankaState& robot_state) {
        // Update time delta
        ros::Time now = ros::Time::now();
        dt = (now - last_update_time).toSec();
        last_update_time = now;
        
        // Clamp dt to reasonable values
        dt = std::min(std::max(dt, 0.001), 0.05);
        
        // Initialize final pose if not set
        if ((last_pose_final.position.x < 0.001 && last_pose_final.position.y < 0.001 && last_pose_final.position.z < 0.001)) {
            transform_current = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());
            Eigen::Vector3d position(transform_current.translation());
            Eigen::Quaterniond orientation(transform_current.rotation());

            final_position_array << position.x(), position.y(), position.z();
            final_orientation_array << orientation.x(), orientation.y(), orientation.z(), orientation.w();
            updateFinalPose();
            return;
        } 

        // Get current robot state
        Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(robot_state.ZeroJacobian.data());
        transform_current = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());
        Eigen::Vector3d position(transform_current.translation());
        Eigen::Quaterniond orientation(transform_current.rotation());

        dq_meas << robot_state.dq[0], robot_state.dq[1], robot_state.dq[2], robot_state.dq[3],
                   robot_state.dq[4], robot_state.dq[5], robot_state.dq[6];

        // Read external force along z-axis
        current_force_z = robot_state.K_F_ext_hat_K[2];  // 3rd element (index 2) is z-force

        // Check force threshold and update saturation logic
        if (current_force_z > force_threshold_z) {  // Force exceeds threshold (more negative)
            if (!z_position_saturated) {
                saturated_z_position = current_position.z();
                z_position_saturated = true;
                ROS_WARN("Z-axis saturated due to excessive force: %.3f N at height: %.3f m", 
                        current_force_z, saturated_z_position);
            }
        } else {
            // Reset saturation if force drops below threshold
            z_position_saturated = false;
        }
        
        current_position = position;
        current_orientation = orientation;

        // Calculate distance to target
        Eigen::Vector3d distance_vector = final_position - current_position;
        double dist = distance_vector.norm();
        
        // Emergency stop check
        if (dist > emergency_stop_distance) {
            ROS_WARN("Emergency stop triggered - distance too large: %.3f m", dist);
            publishZeroVelocities();
            return;
        }
        
        // Publish nextPose status
        std_msgs::Bool msg;
        msg.data = (dist <= dist_to_switch);
        pub_nextPose.publish(msg);

        // Compute twist using VR-optimized approach
        Eigen::VectorXd twist;
        if (vr_mode_enabled) {
            twist = computeVRTwist(current_orientation, current_position, dist);
        } else {
            twist = computeStandardTwist(current_orientation, current_position, dist);
        }

        // Use damped least squares with minimal damping for responsiveness
        Eigen::MatrixXd J = jacobian;
        double lambda = vr_damping_factor; // Minimal damping
        Eigen::MatrixXd JJT = J * J.transpose() + lambda * Eigen::MatrixXd::Identity(6, 6);
        

        //twist[3]=0;
        //twist[4]=0;
        //twist[5]=0;

        //Eigen::VectorXd v_dls =J.transpose() *twist; //JOAO CHANGED THIS, CHANGE BACK

        Eigen::VectorXd v_dls =J.transpose() *  JJT.ldlt().solve(twist);


        // Check for invalid values
        if ((v_dls.array().isNaN() || v_dls.array().isInf()).any()) {
            publishZeroVelocities();
            return;
        }

        if (pub_vel.getNumSubscribers() > 0) {
            // Apply minimal filtering for real-time response
            if (vr_mode_enabled) {
                // Very light filtering to maintain real-time response
                dq = realtime_filter_alpha * dq_prev + (1.0 - realtime_filter_alpha) * v_dls;
            } else {
                // Standard filtering for non-VR mode
                dq = 0.3 * dq_prev + 0.7 * v_dls;
            }
            
            // Apply velocity and acceleration limits
            applyVelocityLimits(dq);
            applyAccelerationLimits(dq);
            
            // Minimal smoothing to reduce high-frequency noise
            if (vr_mode_enabled) {
                dq = applyMinimalSmoothing(dq);
            }
            
            // Store for next iteration
            dq_prev = dq;
            previous_velocity = dq;
            
            // Publish velocity command
            std_msgs::Float64MultiArray vel_msg;
            vel_msg.data.assign(dq.data(), dq.data() + dq.size());
            pub_vel.publish(vel_msg);
        }
    }

    Eigen::VectorXd computeVRTwist(const Eigen::Quaterniond& orientation, const Eigen::Vector3d& position, double dist) {
        // Compute position error
        Eigen::Vector3d position_error = final_position - position;
        
        // VR-optimized linear velocity computation
        Eigen::Vector3d linear_velocity;
        
        if (use_predictive_control && target_velocity.norm() > 0.01) {
            // Predictive control: current error + predicted future position
            Eigen::Vector3d predicted_target = final_position + target_velocity * dt * 2.0; // Look ahead
            Eigen::Vector3d predicted_error = predicted_target - position;
            linear_velocity = vr_following_gain * (0.7 * position_error + 0.3 * predicted_error);
        } else {
            // Standard proportional control with high gain
            linear_velocity = vr_following_gain * position_error;
        }
        
        // Apply precision mode for very small distances
        if (dist < vr_precision_threshold) {
            linear_velocity *= precision_gain_factor;
        }
        
        // Limit linear velocity
        if (linear_velocity.norm() > vr_max_linear_vel) {
            linear_velocity = linear_velocity.normalized() * vr_max_linear_vel;
        }

        // Simplified angular velocity (assuming drawing doesn't require complex orientation changes)
        Eigen::Vector3d angular_velocity;
        Eigen::Quaterniond q_error = final_orientation * orientation.inverse();
        q_error.normalize();
        
        if (q_error.w() < 0) {
            q_error.coeffs() *= -1;
        }
        
        double angle = 2.0 * std::acos(std::abs(q_error.w()));
        if (angle > 1e-4) {
            Eigen::Vector3d axis = q_error.vec().normalized();
            angular_velocity = 2.0 * angle * axis; // Higher angular gain
            
            // Limit angular velocity
            if (angular_velocity.norm() > vr_max_angular_vel) {
                angular_velocity = angular_velocity.normalized() * vr_max_angular_vel;
            }
        } else {
            angular_velocity.setZero();
        }

        // Assemble twist
        Eigen::VectorXd twist(6);
        twist << linear_velocity, angular_velocity;
        return twist;
    }

    Eigen::VectorXd computeStandardTwist(const Eigen::Quaterniond& orientation, const Eigen::Vector3d& position, double dist) {
        // Standard twist computation (original logic)
        Eigen::Vector3d linear_velocity = final_position - position;
        
        // Apply distance-based scaling
        if (dist >= dist_to_switch) {
            linear_velocity = 1.5 * linear_velocity; // Moderate gain
        } else {
            linear_velocity.setZero();
        }

        // Standard orientation control
        Eigen::Quaterniond q_rel = final_orientation * orientation.inverse();
        q_rel.normalize();
        
        double theta = 2.0 * std::acos(std::max(-1.0, std::min(1.0, std::abs(q_rel.w()))));
        
        Eigen::Vector3d angular_velocity;
        if (theta > 1e-4) {
            Eigen::Vector3d axis = q_rel.vec() / std::sin(theta/2.0);
            axis.normalize();
            angular_velocity = 0.5 * theta * axis;
        } else {
            angular_velocity.setZero();
        }

        Eigen::VectorXd twist(6);
        twist << linear_velocity, angular_velocity;
        return twist;
    }

    void applyVelocityLimits(Eigen::VectorXd& dq) {
        double max_vel = vr_mode_enabled ? max_velocity * 1.2 : max_velocity; // Higher limits for VR mode
        
        for (int i = 0; i < 7; i++) {
            if (std::abs(dq(i)) > max_vel) {
                dq(i) = (dq(i) > 0) ? max_vel : -max_vel;
            }
        }
    }

    void applyAccelerationLimits(Eigen::VectorXd& dq) {
        for (int i = 0; i < 7; i++) {
            double acceleration = (dq(i) - previous_velocity(i)) / dt;
            if (std::abs(acceleration) > max_acceleration) {
                double limited_accel = (acceleration > 0) ? max_acceleration : -max_acceleration;
                dq(i) = previous_velocity(i) + limited_accel * dt;
            }
        }
    }

    Eigen::VectorXd applyMinimalSmoothing(const Eigen::VectorXd& new_velocity) {
        // Add to buffer
        velocity_buffer.push_back(new_velocity);
        if (velocity_buffer.size() > max_buffer_size) {
            velocity_buffer.pop_front();
        }
        
        // Simple moving average with small buffer
        Eigen::VectorXd smoothed = Eigen::VectorXd::Zero(7);
        for (const auto& vel : velocity_buffer) {
            smoothed += vel;
        }
        return smoothed / velocity_buffer.size();
    }

    void publishZeroVelocities() {
        std_msgs::Float64MultiArray msg;
        msg.data.assign(7, 0.0);
        pub_vel.publish(msg);
        dq_prev.setZero();
        previous_velocity.setZero();
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "update_pose_end_effector_vr");
    UpdatePoseEndEffector update;
    ros::Rate loop_rate(50); // 100Hz control loop
    
    ROS_INFO("VR Real-time Drawing Controller Started");
    
    while (ros::ok()) {
        ros::spin();
        loop_rate.sleep();
    }
    return 0;
}