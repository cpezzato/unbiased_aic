/*
 * File:   PD_controller.cpp
 * Author: Corrado Pezzato, TU Delft
 *
 * Created on Sept. 29th, 2023
 *
 */

#include "PD.h"
#include <iostream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "PD_controller_single_node");
  // Variables to regulate the flow (Force to read once every 1ms the sensors)
  int count = 0;

  // Object of the class PD which will take care of everything
  PD PD_controller;
  
  ros::Rate rate(1000);

  while (count<1000){
    // Manage all the callbacks and so read sensors
    ros::spinOnce();
      count ++;
    rate.sleep();
  }

  PD_controller.setGoalCurrentState();

  while (ros::ok()){
    // Manage all the callbacks and so read sensors
    ros::spinOnce();
    if ((count!=0)&&(PD_controller.dataReady()==1)){
	    PD_controller.control();
    }
    else
      count ++;
    rate.sleep();
  }
  return 0;
}
