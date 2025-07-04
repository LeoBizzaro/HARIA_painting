#include <iostream>
#include <vector>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <franka_msgs/FrankaState.h>

#include <Eigen/Dense>

class CartesianImpedanceController {
public:
    CartesianImpedanceController(ros::NodeHandle& nh);
    void run();

private:
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void stateCallback(const franka_msgs::FrankaState::ConstPtr& msg);

    ros::NodeHandle nh_;
    ros::Subscriber sub_pose_;
    ros::Subscriber sub_state_;
    ros::Publisher pub_torque_;

    // Robot state
    Eigen::VectorXd q_;
    Eigen::VectorXd dq_;
    Eigen::Matrix<double, 6, 7> jacobian_;
    Eigen::Matrix4d O_T_EE_; // Transformation from base to end-effector

    // Desired pose
    Eigen::Vector3d p_des_;
    Eigen::Quaterniond Q_des_;
    bool has_pose_;

    // Impedance parameters
    Eigen::Matrix<double, 6, 6> K_; // Stiffness matrix
    Eigen::Matrix<double, 6, 6> D_; // Damping matrix
};

CartesianImpedanceController::CartesianImpedanceController(ros::NodeHandle& nh) : nh_(nh), has_pose_(false) {
    // Initialize ROS components
    sub_pose_ = nh_.subscribe("/demo/pose_final", 1, &CartesianImpedanceController::poseCallback, this);
    sub_state_ = nh_.subscribe("/franka_state_controller/franka_states", 1, &CartesianImpedanceController::stateCallback, this);
    pub_torque_ = nh_.advertise<std_msgs::Float64MultiArray>("/fr_joint_torque_controller/command", 1);

    // Initialize Eigen variables
    q_.resize(7);
    dq_.resize(7);
    jacobian_.resize(6, 7);
    O_T_EE_.setIdentity();
    p_des_.setZero();
    Q_des_.setIdentity();

    // Set impedance parameters (Stiffness and Damping)
    K_.setIdentity();
    K_.topLeftCorner(3, 3) << 200, 0, 0, 0, 200, 0, 0, 0, 200; // Translational stiffness
    K_.bottomRightCorner(3, 3) << 20, 0, 0, 0, 20, 0, 0, 0, 20;  // Rotational stiffness

    D_.setIdentity();
    D_.topLeftCorner(3, 3) << 20, 0, 0, 0, 20, 0, 0, 0, 20; // Translational damping
    D_.bottomRightCorner(3, 3) << 5, 0, 0, 0, 5, 0, 0, 0, 5;   // Rotational damping

    ROS_INFO("Cartesian Impedance Controller initialized.");
}

void CartesianImpedanceController::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    p_des_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    Q_des_ = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    has_pose_ = true;
}

// ==========================================================================================
// CORRECTED STATE CALLBACK
// This version correctly interprets the column-major data from the franka_msgs::FrankaState
// ==========================================================================================
void CartesianImpedanceController::stateCallback(const franka_msgs::FrankaState::ConstPtr& msg) {
    // Map joint state data directly from the message to Eigen vectors
    q_ = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(msg->q.data());
    dq_ = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(msg->dq.data());

    // Correctly map the column-major Jacobian from the message to the Eigen matrix
    jacobian_ = Eigen::Map<const Eigen::Matrix<double, 6, 7, Eigen::ColMajor>>(msg->ZeroJacobian.data());

    // Correctly map the column-major Transformation Matrix from the message to the Eigen matrix
    O_T_EE_ = Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::ColMajor>>(msg->O_T_EE.data());
}

void CartesianImpedanceController::run() {
    ros::Rate rate(1000); // Control loop at 1 kHz

    while (ros::ok()) {
        if (has_pose_) {
            // 1. Get current position and orientation from the transformation matrix
            Eigen::Vector3d p_curr = O_T_EE_.topLeftCorner(3, 4).col(3);
            Eigen::Matrix3d R_curr = O_T_EE_.topLeftCorner(3, 3);
            Eigen::Quaterniond Q_curr(R_curr);

            // 2. Compute position error
            Eigen::Vector3d error_p = p_des_ - p_curr;

            // 3. Compute orientation error
            if (Q_des_.coeffs().dot(Q_curr.coeffs()) < 0.0) {
                Q_curr.coeffs() << -Q_curr.coeffs();
            }
            Eigen::Quaterniond error_q = Q_des_ * Q_curr.inverse();
            Eigen::Vector3d error_o(error_q.x() * 2, error_q.y() * 2, error_q.z() * 2);

            // 4. Combine errors into a single 6D vector
            Eigen::Matrix<double, 6, 1> error_x;
            error_x << error_p, error_o;

            // 5. Compute end-effector velocity
            Eigen::Matrix<double, 6, 1> dx = jacobian_ * dq_;

            // 6. Compute desired force/torque (Impedance Law)
            Eigen::Matrix<double, 6, 1> desired_force = K_ * error_x - D_ * dx;

            // 7. Compute joint torques using Jacobian transpose
            Eigen::VectorXd tau_d = jacobian_.transpose() * desired_force;

            // 8. Publish torques
            std_msgs::Float64MultiArray torque_msg;
            torque_msg.data.resize(7);
            for (int i = 0; i < 7; ++i) {
                // Safety: Clamp torques to a reasonable value
                torque_msg.data[i] = std::max(std::min(tau_d(i), 5.0), -5.0);
            }
            pub_torque_.publish(torque_msg);
        }

        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "cartesian_impedance_controller_node");
    ros::NodeHandle nh;

    CartesianImpedanceController controller(nh);
    controller.run();

    return 0;
}