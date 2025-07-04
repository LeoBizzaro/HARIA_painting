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

enum DrawingState {
    APPROACHING,
    IN_CONTACT,
    LIFTING,
    FREE_MOVEMENT
};

class UpdatePoseEndEffector
{
private:
    ros::NodeHandle nh;
    ros::Subscriber franka_states;
    ros::Subscriber pose_final;
    ros::Publisher pub_vel;
    ros::Publisher pub_nextPose;
    ros::Publisher pub_contact_state;  // Publish contact state for monitoring
    
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
    bool vr_mode_enabled;
    double vr_following_gain;
    double vr_max_linear_vel;
    double vr_max_angular_vel;
    double vr_damping_factor;
    
    // Predictive following
    bool use_predictive_control;
    Eigen::Vector3d previous_target_pos;
    Eigen::Vector3d target_velocity;
    ros::Time last_target_time;
    
    // Minimal filtering for real-time response
    double realtime_filter_alpha;
    
    // Safety limits
    double emergency_stop_distance;
    double max_acceleration;
    Eigen::VectorXd previous_velocity;
    
    // VR-specific parameters
    double vr_precision_threshold;
    double precision_gain_factor;
    
    // Velocity smoothing
    std::deque<Eigen::VectorXd> velocity_buffer;
    int max_buffer_size;

    // ===== MARKER DRAWING PARAMETERS =====
    bool marker_mode;                    // Enable marker drawing mode
    bool force_control_enabled;         // Enable force control
    double desired_contact_force;       // Target normal force (N)
    double contact_force_gain;          // P-gain for force control
    double max_contact_force;           // Safety force limit (N)
    double min_contact_threshold;       // Minimum force to consider "contact" (N)
    double contact_loss_threshold;      // Force below which contact is lost (N)
    
    // Drawing state management
    DrawingState current_drawing_state;
    ros::Time state_change_time;
    double state_stability_time;        // Time to wait before state changes
    
    // Force filtering
    std::deque<double> force_buffer;
    int force_buffer_size;
    double filtered_contact_force;
    
    // Marker-specific motion parameters
    double marker_approach_speed;       // Speed when approaching paper
    double marker_drawing_speed_factor; // Speed reduction when in contact
    double marker_lift_speed;           // Speed when lifting from paper
    double marker_z_adjustment_limit;   // Max Z adjustment per cycle (m)
    
    // Contact force history for better control
    double previous_force_error;
    double force_integral_error;
    double force_derivative_gain;
    double force_integral_gain;
    double force_integral_limit;
    
    // Paper height estimation
    double estimated_paper_height;
    bool paper_height_calibrated;
    double paper_height_samples;
    double paper_height_sum;

public:
    UpdatePoseEndEffector()
    {
        pose_final = nh.subscribe("demo/pose_final", 1000, &UpdatePoseEndEffector::poseFinalMessageReceived,this);
        franka_states = nh.subscribe("franka_state_controller/franka_states",1000, &UpdatePoseEndEffector::frankaStateMessageReceived,this);
        pub_vel = nh.advertise<std_msgs::Float64MultiArray>("fr_joint_velocity_controller/command",10);
        pub_nextPose = nh.advertise<std_msgs::Bool>("demo/nextPose", 10);
        pub_contact_state = nh.advertise<std_msgs::Float64MultiArray>("demo/contact_info", 10);
        
        // VR Real-time parameters
        nh.param("vr_mode_enabled", vr_mode_enabled, true);
        nh.param("vr_following_gain", vr_following_gain, 3.0);
        nh.param("vr_max_linear_vel", vr_max_linear_vel, 0.4);
        nh.param("vr_max_angular_vel", vr_max_angular_vel, 1.0);
        nh.param("vr_damping_factor", vr_damping_factor, 0.02);
        
        // Predictive control
        nh.param("use_predictive_control", use_predictive_control, true);
        nh.param("realtime_filter_alpha", realtime_filter_alpha, 0.05);
        
        // Safety parameters
        nh.param("safety_factor", safety_factor, 1.0);
        nh.param("max_velocity", max_velocity, 1.2);
        nh.param("max_acceleration", max_acceleration, 5.0);
        
        // VR precision parameters
        nh.param("vr_precision_threshold", vr_precision_threshold, 0.01);
        nh.param("precision_gain_factor", precision_gain_factor, 0.7);
        
        // ===== MARKER DRAWING PARAMETERS =====
        nh.param("marker_mode", marker_mode, false);
        nh.param("force_control_enabled", force_control_enabled, true);
        nh.param("desired_contact_force", desired_contact_force, 2.5);         // 2.5N target force
        nh.param("contact_force_gain", contact_force_gain, 0.08);              // Gentle P-gain
        nh.param("force_derivative_gain", force_derivative_gain, 0.02);        // D-gain for stability
        nh.param("force_integral_gain", force_integral_gain, 0.01);            // I-gain for steady-state
        nh.param("force_integral_limit", force_integral_limit, 0.005);         // Limit integral windup
        nh.param("max_contact_force", max_contact_force, 8.0);                 // Safety limit
        nh.param("min_contact_threshold", min_contact_threshold, 0.8);         // Contact detection
        nh.param("contact_loss_threshold", contact_loss_threshold, 0.3);       // Contact loss detection
        
        // Motion parameters for marker mode
        nh.param("marker_approach_speed", marker_approach_speed, 0.8);         // 80% of normal speed
        nh.param("marker_drawing_speed_factor", marker_drawing_speed_factor, 0.6); // 60% speed when drawing
        nh.param("marker_lift_speed", marker_lift_speed, 1.2);                // 120% speed when lifting
        nh.param("marker_z_adjustment_limit", marker_z_adjustment_limit, 0.002); // 2mm max adjustment
        nh.param("state_stability_time", state_stability_time, 0.1);           // 100ms state stability
        
        // Adjust safety parameters for marker mode
        if (marker_mode) {
            emergency_stop_distance = 0.15;  // Reduced from 0.4m to 15cm
            max_acceleration = 3.0;          // Gentler accelerations
            vr_max_linear_vel = 0.3;         // Reduced max velocity for precision
            ROS_INFO("Marker drawing mode enabled - adjusted safety parameters");
        } else {
            emergency_stop_distance = 0.4;
        }
        
        dist_to_switch = 0.002;
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
        
        // Initialize velocity smoothing
        max_buffer_size = 3;
        
        // Initialize marker drawing variables
        current_drawing_state = FREE_MOVEMENT;
        state_change_time = ros::Time::now();
        force_buffer_size = 5;  // Small buffer for force filtering
        filtered_contact_force = 0.0;
        previous_force_error = 0.0;
        force_integral_error = 0.0;
        estimated_paper_height = 0.0;
        paper_height_calibrated = false;
        paper_height_samples = 0;
        paper_height_sum = 0.0;
        
        last_update_time = ros::Time::now();
        dt = 1.0/60.0;
        
        ROS_INFO("VR Real-time Drawing Controller Initialized");
        ROS_INFO("Marker Mode: %s, Force Control: %s, Target Force: %.2f N", 
                 marker_mode ? "Enabled" : "Disabled", 
                 force_control_enabled ? "Enabled" : "Disabled",
                 desired_contact_force);
    }

    void poseFinalMessageReceived(const geometry_msgs::PoseStamped& pose) {
        last_pose_final_stamped = pose;
        last_pose_final = last_pose_final_stamped.pose;
        final_orientation_array << last_pose_final.orientation.x, last_pose_final.orientation.y, 
                                   last_pose_final.orientation.z, last_pose_final.orientation.w;
        final_position_array << last_pose_final.position.x, last_pose_final.position.y, last_pose_final.position.z;

        if (use_predictive_control) {
            updateTargetVelocityEstimation();
        }

        updateFinalPose();
    }

    void updateTargetVelocityEstimation() {
        ros::Time now = ros::Time::now();
        double time_delta = (now - last_target_time).toSec();
        
        if (time_delta > 0.001 && time_delta < 0.1) {
            Eigen::Vector3d position_delta = final_position_array - previous_target_pos;
            target_velocity = position_delta / time_delta;
            
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

        transform_final = transform;
        final_position = position;
        final_orientation = orientation;
    }

    double filterContactForce(double raw_force) {
        // Add to circular buffer
        force_buffer.push_back(raw_force);
        if (force_buffer.size() > force_buffer_size) {
            force_buffer.pop_front();
        }
        
        // Calculate filtered force (moving average)
        double sum = 0.0;
        for (const auto& f : force_buffer) {
            sum += f;
        }
        return sum / force_buffer.size();
    }

    void updateDrawingState(double contact_force, const Eigen::Vector3d& position) {
        ros::Time now = ros::Time::now();
        double time_since_state_change = (now - state_change_time).toSec();
        
        DrawingState previous_state = current_drawing_state;
        
        switch(current_drawing_state) {
            case FREE_MOVEMENT:
                // Check if we're close to target Z and force is building up
                if (contact_force > min_contact_threshold && time_since_state_change > state_stability_time) {
                    current_drawing_state = APPROACHING;
                    ROS_INFO("State: FREE_MOVEMENT -> APPROACHING (Force: %.2f N)", contact_force);
                }
                break;
                
            case APPROACHING:
                if (contact_force > desired_contact_force * 0.7 && time_since_state_change > state_stability_time) {
                    current_drawing_state = IN_CONTACT;
                    ROS_INFO("State: APPROACHING -> IN_CONTACT (Force: %.2f N)", contact_force);
                    
                    // Calibrate paper height when first contact is made
                    if (!paper_height_calibrated) {
                        paper_height_sum += position.z();
                        paper_height_samples++;
                        if (paper_height_samples >= 10) {
                            estimated_paper_height = paper_height_sum / paper_height_samples;
                            paper_height_calibrated = true;
                            ROS_INFO("Paper height calibrated: %.4f m", estimated_paper_height);
                        }
                    }
                } else if (contact_force < contact_loss_threshold && time_since_state_change > state_stability_time) {
                    current_drawing_state = FREE_MOVEMENT;
                    ROS_INFO("State: APPROACHING -> FREE_MOVEMENT (Force: %.2f N)", contact_force);
                }
                break;
                
            case IN_CONTACT:
                if (contact_force < contact_loss_threshold && time_since_state_change > state_stability_time) {
                    current_drawing_state = LIFTING;
                    ROS_INFO("State: IN_CONTACT -> LIFTING (Force: %.2f N)", contact_force);
                } else if (contact_force > max_contact_force) {
                    current_drawing_state = LIFTING;
                    ROS_WARN("State: IN_CONTACT -> LIFTING (Excessive force: %.2f N)", contact_force);
                }
                break;
                
            case LIFTING:
                if (contact_force < contact_loss_threshold * 0.5 && time_since_state_change > state_stability_time) {
                    current_drawing_state = FREE_MOVEMENT;
                    ROS_INFO("State: LIFTING -> FREE_MOVEMENT (Force: %.2f N)", contact_force);
                }
                break;
        }
        
        if (current_drawing_state != previous_state) {
            state_change_time = now;
        }
    }

    double computeForceControlAdjustment(double contact_force) {
        if (!force_control_enabled || current_drawing_state == FREE_MOVEMENT) {
            return 0.0;
        }
        
        double force_error = desired_contact_force - contact_force;
        double force_derivative = (force_error - previous_force_error) / dt;
        
        // Update integral term with anti-windup
        force_integral_error += force_error * dt;
        force_integral_error = std::max(-force_integral_limit, 
                                       std::min(force_integral_limit, force_integral_error));
        
        // PID control for Z adjustment
        double z_adjustment = contact_force_gain * force_error + 
                             force_derivative_gain * force_derivative +
                             force_integral_gain * force_integral_error;
        
        // Limit adjustment magnitude
        z_adjustment = std::max(-marker_z_adjustment_limit, 
                               std::min(marker_z_adjustment_limit, z_adjustment));
        
        previous_force_error = force_error;
        return z_adjustment;
    }

    void frankaStateMessageReceived(const franka_msgs::FrankaState& robot_state) {
        ros::Time now = ros::Time::now();
        dt = (now - last_update_time).toSec();
        last_update_time = now;
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
        
        current_position = position;
        current_orientation = orientation;

        // Get external forces for marker control
        Eigen::Map<const Eigen::Matrix<double, 6, 1>> f_ext(robot_state.O_F_ext_hat_K.data());
        double raw_contact_force = std::abs(f_ext(2)); // Z-component (normal to paper)
        filtered_contact_force = filterContactForce(raw_contact_force);
        
        // Update drawing state if in marker mode
        if (marker_mode) {
            updateDrawingState(filtered_contact_force, position);
        }

        // Calculate distance to target
        Eigen::Vector3d distance_vector = final_position - current_position;
        double dist = distance_vector.norm();
        
        // Emergency stop check (adjusted for marker mode)
        if (dist > emergency_stop_distance) {
            ROS_WARN("Emergency stop triggered - distance too large: %.3f m", dist);
            publishZeroVelocities();
            return;
        }
        
        // Publish nextPose status
        std_msgs::Bool msg;
        msg.data = (dist <= dist_to_switch);
        pub_nextPose.publish(msg);

        // Publish contact information for monitoring
        if (marker_mode && pub_contact_state.getNumSubscribers() > 0) {
            std_msgs::Float64MultiArray contact_msg;
            contact_msg.data.resize(4);
            contact_msg.data[0] = static_cast<double>(current_drawing_state);
            contact_msg.data[1] = filtered_contact_force;
            contact_msg.data[2] = desired_contact_force;
            contact_msg.data[3] = position.z() - estimated_paper_height;
            pub_contact_state.publish(contact_msg);
        }

        // Compute twist
        Eigen::VectorXd twist;
        if (marker_mode) {
            twist = computeMarkerTwist(current_orientation, current_position, dist, f_ext);
        } else if (vr_mode_enabled) {
            twist = computeVRTwist(current_orientation, current_position, dist);
        } else {
            twist = computeStandardTwist(current_orientation, current_position, dist);
        }

        // Use damped least squares
        Eigen::MatrixXd J = jacobian;
        double lambda = marker_mode ? vr_damping_factor * 0.5 : vr_damping_factor; // Less damping for marker mode
        Eigen::MatrixXd JJT = J * J.transpose() + lambda * Eigen::MatrixXd::Identity(6, 6);
        Eigen::VectorXd v_dls = J.transpose() * JJT.ldlt().solve(twist);

        if ((v_dls.array().isNaN() || v_dls.array().isInf()).any()) {
            publishZeroVelocities();
            return;
        }

        if (pub_vel.getNumSubscribers() > 0) {
            // Apply filtering based on mode
            if (marker_mode) {
                // Gentler filtering for marker mode
                dq = realtime_filter_alpha * 0.5 * dq_prev + (1.0 - realtime_filter_alpha * 0.5) * v_dls;
            } else if (vr_mode_enabled) {
                dq = realtime_filter_alpha * dq_prev + (1.0 - realtime_filter_alpha) * v_dls;
            } else {
                dq = 0.3 * dq_prev + 0.7 * v_dls;
            }
            
            applyVelocityLimits(dq);
            applyAccelerationLimits(dq);
            
            if (vr_mode_enabled || marker_mode) {
                dq = applyMinimalSmoothing(dq);
            }
            
            dq_prev = dq;
            previous_velocity = dq;
            
            std_msgs::Float64MultiArray vel_msg;
            vel_msg.data.assign(dq.data(), dq.data() + dq.size());
            pub_vel.publish(vel_msg);
        }
    }

    Eigen::VectorXd computeMarkerTwist(const Eigen::Quaterniond& orientation, 
                                       const Eigen::Vector3d& position, 
                                       double dist,
                                       const Eigen::VectorXd& f_ext) {
        
        // Compute base position error
        Eigen::Vector3d position_error = final_position - position;
        Eigen::Vector3d linear_velocity;
        
        // Apply state-dependent speed scaling
        double speed_factor = 1.0;
        switch(current_drawing_state) {
            case APPROACHING:
                speed_factor = marker_approach_speed;
                break;
            case IN_CONTACT:
                speed_factor = marker_drawing_speed_factor;
                break;
            case LIFTING:
                speed_factor = marker_lift_speed;
                break;
            case FREE_MOVEMENT:
                speed_factor = 1.0;
                break;
        }
        
        if (use_predictive_control && target_velocity.norm() > 0.01) {
            Eigen::Vector3d predicted_target = final_position + target_velocity * dt * 2.0;
            Eigen::Vector3d predicted_error = predicted_target - position;
            linear_velocity = vr_following_gain * (0.7 * position_error + 0.3 * predicted_error);
        } else {
            linear_velocity = vr_following_gain * position_error;
        }
        
        // Apply speed scaling
        linear_velocity *= speed_factor;
        
        // Force control adjustment for Z-axis
        if (force_control_enabled && (current_drawing_state == APPROACHING || current_drawing_state == IN_CONTACT)) {
            double z_adjustment = computeForceControlAdjustment(filtered_contact_force);
            linear_velocity(2) = z_adjustment;  // Override Z component with force control
            
            // Reduce XY movement when in contact for better drawing quality
            if (current_drawing_state == IN_CONTACT) {
                linear_velocity(0) *= 0.8;
                linear_velocity(1) *= 0.8;
            }
        }
        
        // Apply precision mode for very small distances
        if (dist < vr_precision_threshold) {
            linear_velocity *= precision_gain_factor;
        }
        
        // Limit linear velocity (marker-adjusted limits)
        double max_lin_vel = marker_mode ? vr_max_linear_vel * 0.8 : vr_max_linear_vel;
        if (linear_velocity.norm() > max_lin_vel) {
            linear_velocity = linear_velocity.normalized() * max_lin_vel;
        }

        // Simplified angular velocity for drawing (maintain orientation stability)
        Eigen::Vector3d angular_velocity;
        Eigen::Quaterniond q_error = final_orientation * orientation.inverse();
        q_error.normalize();
        
        if (q_error.w() < 0) {
            q_error.coeffs() *= -1;
        }
        
        double angle = 2.0 * std::acos(std::abs(q_error.w()));
        if (angle > 1e-4) {
            Eigen::Vector3d axis = q_error.vec().normalized();
            double angular_gain = marker_mode ? 1.5 : 2.0; // Gentler angular control for markers
            angular_velocity = angular_gain * angle * axis;
            
            double max_ang_vel = marker_mode ? vr_max_angular_vel * 0.7 : vr_max_angular_vel;
            if (angular_velocity.norm() > max_ang_vel) {
                angular_velocity = angular_velocity.normalized() * max_ang_vel;
            }
        } else {
            angular_velocity.setZero();
        }

        Eigen::VectorXd twist(6);
        twist << linear_velocity, angular_velocity;
        return twist;
    }

    Eigen::VectorXd computeVRTwist(const Eigen::Quaterniond& orientation, const Eigen::Vector3d& position, double dist) {
        // Original VR twist computation (unchanged)
        Eigen::Vector3d position_error = final_position - position;
        Eigen::Vector3d linear_velocity;
        
        if (use_predictive_control && target_velocity.norm() > 0.01) {
            Eigen::Vector3d predicted_target = final_position + target_velocity * dt * 2.0;
            Eigen::Vector3d predicted_error = predicted_target - position;
            linear_velocity = vr_following_gain * (0.7 * position_error + 0.3 * predicted_error);
        } else {
            linear_velocity = vr_following_gain * position_error;
        }
        
        if (dist < vr_precision_threshold) {
            linear_velocity *= precision_gain_factor;
        }
        
        if (linear_velocity.norm() > vr_max_linear_vel) {
            linear_velocity = linear_velocity.normalized() * vr_max_linear_vel;
        }

        Eigen::Vector3d angular_velocity;
        Eigen::Quaterniond q_error = final_orientation * orientation.inverse();
        q_error.normalize();
        
        if (q_error.w() < 0) {
            q_error.coeffs() *= -1;
        }
        
        double angle = 2.0 * std::acos(std::abs(q_error.w()));
        if (angle > 1e-4) {
            Eigen::Vector3d axis = q_error.vec().normalized();
            angular_velocity = 2.0 * angle * axis;
            
            if (angular_velocity.norm() > vr_max_angular_vel) {
                angular_velocity = angular_velocity.normalized() * vr_max_angular_vel;
            }
        } else {
            angular_velocity.setZero();
        }

        Eigen::VectorXd twist(6);
        twist << linear_velocity, angular_velocity;
        return twist;
    }

    Eigen::VectorXd computeStandardTwist(const Eigen::Quaterniond& orientation, const Eigen::Vector3d& position, double dist) {
        // Original standard twist computation (unchanged)
        Eigen::Vector3d linear_velocity = final_position - position;
        
        if (dist >= dist_to_switch) {
            linear_velocity = 1.5 * linear_velocity;
        } else {
            linear_velocity.setZero();
        }

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
        double max_vel = max_velocity;
        if (vr_mode_enabled) {
            max_vel *= 1.2;
        }
        if (marker_mode && current_drawing_state == IN_CONTACT) {
            max_vel *= 0.8; // Reduce velocity when marker is in contact
        }
        
        for (int i = 0; i < 7; i++) {
            if (std::abs(dq(i)) > max_vel) {
                dq(i) = (dq(i) > 0) ? max_vel : -max_vel;
            }
        }
    }

    void applyAccelerationLimits(Eigen::VectorXd& dq) {
        double max_accel = marker_mode ? max_acceleration * 0.8 : max_acceleration;
        
        for (int i = 0; i < 7; i++) {
            double acceleration = (dq(i) - previous_velocity(i)) / dt;
            if (std::abs(acceleration) > max_accel) {
                double limited_accel = (acceleration > 0) ? max_accel : -max_accel;
                dq(i) = previous_velocity(i) + limited_accel * dt;
            }
        }
    }

    Eigen::VectorXd applyMinimalSmoothing(const Eigen::VectorXd& new_velocity) {
        velocity_buffer.push_back(new_velocity);
        if (velocity_buffer.size() > max_buffer_size) {
            velocity_buffer.pop_front();
        }
        
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
        
        // Reset force control integral term
        force_integral_error = 0.0;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "update_pose_end_effector_marker");
    UpdatePoseEndEffector update;
    ros::Rate loop_rate(50); // 50Hz control loop for marker precision
    
    ROS_INFO("VR Real-time Drawing Controller with Marker Support Started");
    
    while (ros::ok()) {
        ros::spin();
        loop_rate.sleep();
    }
    return 0;
}