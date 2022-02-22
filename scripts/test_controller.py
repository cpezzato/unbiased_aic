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
        self._action_msg.ref_position.data = np.array([0.5, 0.1, 0.0, -1.501, 0.0, 1.8675, 0.0])
        self._action_msg.ref_velocity.data = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    def run(self):
        t = 0.0
        a = 0.7
        w = 1
        while not rospy.is_shutdown():
            t += 0.01
            p0 = a * np.cos(w * t)
            v0 = - a * w * np.sin(w * t)
            self._action_msg.ref_position.data[0] = p0
            self._action_msg.ref_velocity.data[0] = v0
            self._action_pub.publish(self._action_msg)
            self._rate.sleep()


if __name__ == "__main__":
    myTestController = TestController()
    try:
        myTestController.run()
    except rospy.ROSInterruptException:
        pass
