#include <ros/ros.h>
#include <franka_msgs/FrankaState.h>
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <franka/robot.h>
#include <franka/model.h>
#include <array>
#include <iostream>
#include <cmath>
#include <functional>
#include <franka/duration.h>
#include <franka/exception.h>

class ImpedanceControl {
private:
    ros::NodeHandle nh;
    ros::Subscriber franka_states;
    ros::Publisher pub_torque;

    Eigen::Vector3d initial_position;
    Eigen::Quaterniond initial_orientation;
    bool is_initialized = false;
    
    franka::Robot robot;
    franka::Model model;
    franka::RobotState current_state;

    Eigen::Matrix<double, 3, 3> K, D;
    double velocity_gain;

    Eigen::Affine3d initial_transform;
    Eigen::Vector3d position_d;
    Eigen::Quaterniond orientation_d;
    Eigen::MatrixXd stiffness; 
    Eigen::MatrixXd damping;

public:
    ImpedanceControl() : robot("192.168.1.123"), model(robot.loadModel()) {
        franka_states = nh.subscribe("franka_state_controller/franka_states", 1000, &ImpedanceControl::frankaStateMessageReceived, this);
        pub_torque = nh.advertise<std_msgs::Float64MultiArray>("fr_joint_torque_controller/command", 10);
        
        // Compliance parameters
        const double translational_stiffness{150.0}; // 150.0
        const double rotational_stiffness{10.0}; // 10.0
        
        stiffness = Eigen::MatrixXd::Zero(6, 6);
        damping = Eigen::MatrixXd::Zero(6, 6);
        stiffness.topLeftCorner(3, 3) = translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
        stiffness.bottomRightCorner(3, 3) = rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
        damping.topLeftCorner(3, 3) = 2.0 * sqrt(translational_stiffness) * Eigen::MatrixXd::Identity(3, 3); //2.0
        damping.bottomRightCorner(3, 3) = 2.0 * sqrt(rotational_stiffness) * Eigen::MatrixXd::Identity(3, 3); //2.0

        // set collision behavior
        robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

        franka::RobotState initial_state = robot.readOnce();
 
        // equilibrium point is the initial position
        initial_transform = Eigen::Matrix4d::Map(initial_state.O_T_EE.data());
        position_d = initial_transform.translation();
        orientation_d = initial_transform.rotation();
    }

    void frankaStateMessageReceived(const franka_msgs::FrankaState& robot_state) {
        for (int i = 0; i < 7; i++) current_state.q[i] = robot_state.q[i];
        for (int i = 0; i < 7; i++) current_state.dq[i] = robot_state.dq[i];
        for (int i = 0; i < 16; i++) current_state.O_T_EE[i] = robot_state.O_T_EE[i];

        std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, current_state);
        std::array<double, 7> coriolis_array = model.coriolis(current_state);

        // convert to Eigen
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
        Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
        Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
        Eigen::Vector3d position(transform.translation());
        Eigen::Quaterniond orientation(transform.rotation());

        // compute error to desired equilibrium pose
        // position error
        Eigen::Matrix<double, 6, 1> error;
        error.head(3) << position - position_d;
 
        // orientation error
        // "difference" quaternion
        if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
          orientation.coeffs() << -orientation.coeffs();
        }
        // "difference" quaternion
        Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
        error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
        // Transform to base frame
        error.tail(3) << -transform.rotation() * error.tail(3);
 
        // compute control
        Eigen::VectorXd tau_task(7), tau_d(7);
 
        // Spring damper system with damping ratio=1
        tau_task << jacobian.transpose() * (-stiffness * error - damping * (jacobian * dq));
        tau_d << tau_task + coriolis;

        std::array<double, 7> tau_d_array{};
        Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;

        std_msgs::Float64MultiArray msg;
        msg.layout.dim.push_back(std_msgs::MultiArrayDimension());  // Definizione della dimensione
        msg.layout.dim[0].size = 7;
        msg.layout.dim[0].stride = 1;
        msg.data.insert(msg.data.end(), tau_d_array.begin(), tau_d_array.end());

        pub_torque.publish(msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "impedance_control");
    ImpedanceControl control;
    ros::spin();
    return 0;
}
