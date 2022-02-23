#!/usr/bin/env python

import rospy
import numpy as np
import time

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

    def run(self):
        t = 0.0
        while not rospy.is_shutdown():
            if t > 0.0 and t < 2.0:
                self._action_msg.ref_position.data = np.array([-0.0, -0.917, -0.0, -2.619, 0.0349, 1.719, -0.0])
                self._action_msg.ref_velocity.data = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            if t > 2.0 and t < 6.0:
                self._action_msg.ref_position.data = np.array([0.431, -0.126, 0.0516, -2.537, -0.0196, 2.425, -0.0])
                self._action_msg.ref_velocity.data = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            if t > 6.0 and t < 10.0:
                self._action_msg.ref_position.data = np.array([0.353, 0.266, 0.0417, -2.594, 0.0314, 2.895, -0.0]) 
                self._action_msg.ref_velocity.data = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            if t > 10.0 and t < 12.0:
                self._action_msg.ref_position.data = np.array([0.431, -0.126, 0.0516, -2.537, -0.0196, 2.425, -0.0])
                self._action_msg.ref_velocity.data = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            if t > 12.0:
                t =0   
            t += 0.01
            self._action_pub.publish(self._action_msg)
            self._rate.sleep()


if __name__ == "__main__":
    myTestController = TestController()
    try:
        myTestController.run()
    except rospy.ROSInterruptException:
        pass
