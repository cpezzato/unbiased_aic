/*
 * File:   uAIC_controller.cpp
 * Author: Corrado Pezzato, TU Delft, DCSC
 *
 * Created on Sept. 28th, 2021
 *
 */

#include "uAIC.h"
#include <iostream>

// Constant for class uAIC constructor to define which robot to control
const int robot = 1;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "uAIC_controller_single_node");
  // Variables to regulate the flow (Force to read once every 1ms the sensors)
  int count = 0;
  int cycles = 0;
  // Variable for desired position, set here the goal for the Panda for each joint
  std::vector<double> desiredPos1(7);

  // Set desired position to be achieved at the start, without having any goal from a publisher
  desiredPos1[0] = 0;
  desiredPos1[1] = -0.4;
  desiredPos1[2] = 0;
  desiredPos1[3] = -2;
  desiredPos1[4] = 0;
  desiredPos1[5] = 1.5;
  desiredPos1[6] = 0.0;

  // Object of the class uAIC which will take care of everything
  uAIC uAIC_controller(robot);
  // Set desired position in the uAIC class
  uAIC_controller.setGoal(desiredPos1);
  // Set the current joint position as goal for the robot to keep it
  // Main loop
  ros::Rate rate(1000);

  while (count<1000){
    // Manage all the callbacks and so read sensors
    ros::spinOnce();
      count ++;
    rate.sleep();
  }

//uAIC_controller.setGoalCurrentState();

  while (ros::ok()){
    // Manage all the callbacks and so read sensors
    ros::spinOnce();
    if ((count!=0)&&(uAIC_controller.dataReady()==1)){
	    uAIC_controller.minimiseF();
    }
    else
      count ++;
    rate.sleep();
  }
  return 0;
}
