#!/usr/bin/env python

import rospy
import actionlib
import numpy as np
import time
from franka_gripper.msg import MoveAction, MoveGoal, GraspAction, GraspGoal
from unbiased_aic.msg import reference

class TestController(object):
    def __init__(self):
        rospy.init_node('test_aic_controller_node')
        self._rate = rospy.Rate(100)
        self._action_pub = rospy.Publisher(
            '/desired_state', 
            reference, queue_size=10
        )
        self._action_msg = reference()
        self._action_msg.ref_position.data = np.array([-0.0, -0.917, -0.0, -2.619, 0.0349, 1.719, -0.0])
        self._action_msg.ref_velocity.data = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        self.client = actionlib.SimpleActionClient('franka_gripper/move', MoveAction)
        self.client.wait_for_server()
        print("Found grasp server")
        # Creates a goal to send to the action server.
        self.goal = MoveGoal()

    def run(self):
        t = 0.0
        while not rospy.is_shutdown():
            if t > 0.0 and t < 2.0:
                self._action_msg.ref_position.data = np.array([-0.0, -0.4, -0.0, -2, 0.0, 1.5, -0.0])
                self._action_msg.ref_velocity.data = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            if t > 2.0 and t < 8.0:
                self._action_msg.ref_position.data = np.array([-0.005562202513847279, 0.08350535817522751, 0.01295866326151187, -2.6517260061230576, -0.0036131685750793595, 2.6451942547162375, 0.9056253972195016])
                self._action_msg.ref_velocity.data = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            if t > 8.0:
		self.goal.width = 0.005
                self.goal.speed = 0.03
		self.client.send_goal(self.goal)
		# Waits for the server to finish performing the action.
	    if t > 11.0:
            	self._action_msg.ref_position.data = np.array([-0.0, -0.4, -0.0, -2, 0.0, 1.5, 0.9056253972195016])
                self._action_msg.ref_velocity.data = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            t += 0.01
            self._action_pub.publish(self._action_msg)
            self._rate.sleep()

    def test_pose(self):
	self._action_msg.ref_position.data = np.array([0.020104820503887597, 0.0055614423509567695, -0.034332948067742064, -2.7544014614506773, -0.1306302901054145, 2.779634562334175, 0.951976710015627])
        self._action_msg.ref_velocity.data = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
	self._action_pub.publish(self._action_msg)
        self._rate.sleep()
	
    def gripper_control(self, param):
        if param == "grasp":
            self.goal.width = 0.005
            self.goal.speed = 0.03
        else:
            self.goal.width = 0.08
            self.goal.speed = 0.03
         # Send the goal to the action server.
        self.client.send_goal(self.goal)
         # Waits for the server to finish performing the action.
        self.client.wait_for_result()

if __name__ == "__main__":
    myTestController = TestController()
    try:
        myTestController.gripper_control("release")
        myTestController.run()
        #myTestController.test_pose()
    except rospy.ROSInterruptException:
        pass
