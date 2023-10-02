/*
 * File:   AIC.cpp
 * Author: Corrado Pezzato, TU Delft, DCSC
 *
 * Created on Sept. 29th, 2021
 *
 * Class to perform active inference contro
 * Definition of the methods contained in AIC.h
 *
 */

#include "PD.h"

const double kDefaultStiffnessData[7] = {600, 600, 600, 600, 250, 150, 50};
const Eigen::Matrix<double, 7, 1> PD::kDefaultStiffness =
    Eigen::Matrix<double, 7, 1>(kDefaultStiffnessData);

const double kDefaultDampingData[7] = {50, 50, 50, 20, 20, 20, 10};
const Eigen::Matrix<double, 7, 1> PD::kDefaultDamping = Eigen::Matrix<double, 7, 1>(kDefaultDampingData);

const double PD::kDefaultFilterCoeff = 0.2;

const double kDefaultIData[7] = {5, 5, 5, 2, 2, 2, 1};
const Eigen::Matrix<double, 7, 1> PD::kDefaultI = Eigen::Matrix<double, 7, 1>(kDefaultIData);

const double velErrorCumMaxDefaultData[7] = {0.5, 0.5, 0.5, 0.2, 0.2, 0.2, 0.1};
const Eigen::Matrix<double, 7, 1> PD::velErrorCumMaxDefault = Eigen::Matrix<double, 7, 1>(velErrorCumMaxDefaultData);

const double velErrorCumMinDefaultData[7] = {-0.5, -0.5, -0.5, -0.2, -0.2, -0.2, -0.1};
const Eigen::Matrix<double, 7, 1> PD::velErrorCumMinDefault = Eigen::Matrix<double, 7, 1>(velErrorCumMinDefaultData);

// Constructor which takes as argument the publishers and initialises the private ones in the class
PD::PD(){
  // Initialize the variables for thr PD
  PD::initVariables();
    // Torque publisher
    torque_pub = nh.advertise<std_msgs::Float64MultiArray>("/panda_joint_effort_controller/command", 20);
    filter_pub = nh.advertise<std_msgs::Float64MultiArray>("/filtered_state", 20);
    // Listener joint states
    sensorSub = nh.subscribe("/franka_state_controller/joint_states", 1, &PD::jointStatesCallback, this);

    // Listener to goals
    goal_vel_sub = nh.subscribe("/desired_vel", 5, &PD::setDesiredVel, this);

}
PD::~PD(){}

// Method to set the current goal from topic, this is the input to the controller
void PD::setDesiredVel(const std_msgs::Float64MultiArray::ConstPtr& msg){
  for( int i = 0; i < 7; i++ ) {
    dq_d(i) = msg->data[i];
  }
}

void PD::jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  // Save joint values
  for( int i = 0; i < 7; i++ ) {
    jointPos(i) = msg->position[i];
    jointVel(i) = msg->velocity[i];
  }
  
  if (dataReceived == 0){
    dataReceived = 1;
  }
}

void PD::initVariables(){

  // Initialize parameters to default 
  K_p = kDefaultStiffness.asDiagonal();
  K_d = kDefaultDamping.asDiagonal();
  K_i = kDefaultI.asDiagonal();
  filter_coeff_ = kDefaultFilterCoeff;

  vel_error_cum_max_ = velErrorCumMaxDefault;
  vel_error_cum_min_ = velErrorCumMinDefault;

  // Support variable
  // nh.getParam("max_i", max_i);

  // Initialize control actions
  u << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  // Integration step
  h = 0.001;
  // Resize the data for the published message
  torque_command.data.resize(7);
  filtered_vel.data.resize(7);
}

// PD control
void PD::control(){
  // Update filter
  dq_filtered = ema_filter(dq_filtered, jointVel, filter_coeff_, false);

  vel_error = (dq_d - dq_filtered);
  vel_error_cum += vel_error;
  vel_error_cum = vel_error_cum.cwiseMax(vel_error_cum_min_).cwiseMin(vel_error_cum_max_);
  
  u << 0.1*K_p * vel_error; // + K_i * vel_error_cum; // - K_d * dq_filtered;

  // Set the toques from u and publish
  for (int i=0;i<7;i++){
    torque_command.data[i] = u(i);
    filtered_vel.data[i] = dq_filtered(i);
  }

  // Publishing
  //torque_pub.publish(torque_command);
  filter_pub.publish(filtered_vel);
}

// Method to control if the joint states have been received already,
// used in the main function
int PD::dataReady(){
  if(dataReceived==1)
    return 1;
  else
    return 0;
}

void PD::setGoalCurrentState(){
  for(int i=0; i<7; i++){
    dq_d(i) = 0;
  }
}

// TODO: add the update functions, perhaps reading from ros parameter server