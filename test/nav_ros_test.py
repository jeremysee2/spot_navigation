#!/usr/bin/env python3
PKG = "nav_ros"
NAME = "nav_ros_test"
SUITE = "nav_ros_test.TestSuiteNavROS"

import time
import pickle
import unittest

import rospy
import actionlib
from rosservice import get_service_class_by_name

from spot_msgs.msg import WorldObjectArray


class TestNavROS(unittest.TestCase):
    def setUp(self):
        pass

    def test_something(self):
        self.assertTrue(True)

    def tearDown(self):
        pass


class TestSuiteNavROS(unittest.TestSuite):
    def __init__(self):
        super(TestSuiteNavROS, self).__init__()

        self.loader = unittest.TestLoader()
        self.addTest(self.loader.loadTestsFromTestCase(TestNavROS))


if __name__ == "__main__":
    import rosunit

    rospy.init_node(NAME, anonymous=True)
    rosunit.unitrun(PKG, NAME, SUITE)
