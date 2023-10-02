/*
 * File:   PD.h
 * Author: Corrado Pezzato, TU Delft
 *
 * Created on Sept. 29th, 2023
 *
 * Class for PD control
 *
 */

#ifndef PD_H
#define PD_H
#define _USE_MATH_DEFINES

#include "ros/ros.h"
#include <vector>
#include <cmath>
#include <sensor_msgs/JointState.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/PoseStamped.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <stdlib.h>

template <typename T>
T ema_filter(const T& value_f, const T& value, double alpha,
             bool rounding = false, double threshold = 1e-20);

// Template definition for the general case, i.e. Eigen::Matrix
template <typename EigenMatrix>
inline EigenMatrix ema_filter(const EigenMatrix& value_f,
                              const EigenMatrix& value, double alpha,
                              bool rounding, double threshold) {
  return value_f.binaryExpr(
      value, [alpha, rounding, threshold](const auto v_f, const auto v) {
        return ema_filter(v_f, v, alpha, rounding, threshold);
      });
}

// Template specialization for double
template <>
inline double ema_filter<double>(const double& value_f, const double& value,
                                 double alpha, bool rounding,
                                 double threshold) {
  if (rounding && std::abs(value - value_f) < threshold) {
    return value;
  }
  return alpha * value + (1 - alpha) * value_f;
}

// Class PD to hanle the subscribers and the publishers for the active inference controller
class PD
{
public:
  // Constructor and destructor
  PD();
  ~PD();

  static const Eigen::Matrix<double, 7, 1> kDefaultStiffness;
  static const Eigen::Matrix<double, 7, 1> kDefaultDamping;
  static const Eigen::Matrix<double, 7, 1> kDefaultI;
  static const Eigen::Matrix<double, 7, 1> velErrorCumMaxDefault;
  static const Eigen::Matrix<double, 7, 1> velErrorCumMinDefault;
  static const double kDefaultFilterCoeff;

  void control();
  void setStiffness(const Eigen::Matrix<double, 7, 1> &stiffness);
  void setDamping(const Eigen::Matrix<double, 7, 1> &damping);
  void setVelErrorCumMax(const Eigen::Matrix<double, 7, 1> &vel_error_cum_max);
  void setVelErrorCumMin(const Eigen::Matrix<double, 7, 1> &vel_error_cum_min);
  void setKi(const Eigen::Matrix<double, 7, 1> &K_i);
  void setFilter(const double filter_coeff);

  // Callback to handle the proprioceptive sensory data from the topic /joint_states published at 1kHz
  void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);
  // Method to set the necessary variables once the constructor is called
  void initVariables();
  // Support method to control the program flow. Data ready returns one when the encoders has been read
  int dataReady();
  void setGoalCurrentState();
  // Getting trajectory
  void setDesiredVel(const std_msgs::Float64MultiArray::ConstPtr& msg);

private:

  Eigen::Matrix<double, 7, 1> dq_filtered, vel_error_cum_max_, vel_error_cum_min_, vel_error, vel_error_cum;
  double filter_coeff_;
  // Controller parameters, PID like control law. Diagonal matrices
  Eigen::DiagonalMatrix<double, 7> K_p, K_i, K_d;
  Eigen::Matrix<double, 7, 1> jointPos, jointVel;
  // Desired robot's velocity
  Eigen::Matrix<double, 7, 1> dq_d;
  // Control actions,  column vector of 7 elements, and integral gain
  Eigen::Matrix<double, 7, 1> u;
  double h;
  // Support variable to control the flow of the script
  int dataReceived;
  // ROS related Variables, node handle
  ros::NodeHandle nh;
  // Publishers for joint torques to the topics /panda_joint*_controller/command, and the free-energy
  ros::Publisher torque_pub;
  ros::Publisher filter_pub;
  // Subscriber for proprioceptive sensors (i.e. from joint_states) and camera (i.e. aruco_single/pose)
  ros::Subscriber sensorSub;
  // Definition of variables in order to publish torques
  std_msgs::Float64MultiArray torque_command, filtered_vel;
  // Getting goal
  ros::Subscriber goal_vel_sub;
};

#endif
