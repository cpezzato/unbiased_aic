# Unbiased active inference for torque control

![Alt Text](docs/panda_banana.gif)

This package contains implementations of the active inference controller (AIC) from [1] and the unbiased active inference controller. The controllers are designed and tuned for satisfactory behavior of a Franka Emika Panda. 

### Pre-requisites 
This package relies on joint states readings and torque commands for the Panda arm. Provided a goal through the ````/desired_state topic````, the active inference controllers compute the required torques and publish them to the topic ````/panda_joint_effort_controller/command````. Thus two requirements:

1. Franka ROS is installed and configured
2. A high frequency torque interface is available such that the published torques by the active inference are forwarded to the joints

## How to install
Simply clone this repository and build it in your workspace.

````
mkdir uaic_ws/src
cd src
git clone git@github.com:cpezzato/unbiased_aic.git
cd ..
catkin build
````

## Examples of how to use

*Note: Be sure that the Panda is publishing joint states at ````/franka_state_controller/joint_states````*


Given an available torque interface with the Panda arm which listens to the topic ````/panda_joint_effort_controller/command````, the controllers (either AIC or uAIC) can be launched with
````
roslaunch unbiased_aic $controller$.launch
````

Available controllers are ````aic```` and ````unbiased_aic````.  

After that the robot will move to a central position and try to maintain it. To make the robot move you need to run one of the available Python scripts to send desired goal states to be achieved. For a simple tracking test of sinusoidal waves for the first three joints run:

````
rosrun unbiased_aic $test_controller.py$
````

You can write your own examples in Python following the examples in ````/scripts````. The tuning of the controllers can be done in the ````.yaml```` files in the ````\config```` folder. 

