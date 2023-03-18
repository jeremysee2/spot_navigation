PKG = "spot_nav"
NAME = "spot_nav_test"
SUITE = "spot_nav_test.TestSuiteSpotNav"

import unittest
import rospy

from geometry_msgs.msg import PoseStamped
from spot_navigation.spot_nav import SpotNav


class TestSpotNav(unittest.TestCase):
    def setUp(self):
        self._spot_nav = SpotNav()

    def test_something(self):
        self.assertTrue(True)

    def tearDown(self):
        pass


class TestSuiteSpotNav(unittest.TestSuite):
    def __init__(self):
        super(TestSuiteSpotNav, self).__init__()

        self.loader = unittest.TestLoader()
        self.addTest(self.loader.loadTestsFromTestCase(TestSpotNav))


if __name__ == "__main__":
    import rosunit

    rosunit.unitrun(PKG, NAME, SUITE)
