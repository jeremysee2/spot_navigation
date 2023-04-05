#! /usr/bin/env python3
import rospy
from actionlib import SimpleActionClient

from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import (
    MoveBaseAction,
    MoveBaseGoal,
    MoveBaseResult,
    MoveBaseFeedback,
)


class SpotPathPlan(object):
    def send_goal(self, x: float, y: float, z: float, w: float) -> MoveBaseResult:
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.z = z
        goal.target_pose.pose.orientation.w = w

        self._ac.send_goal(goal)
        self._ac.wait_for_result()

        return self._ac.get_result()

    def main(self):
        rospy.init_node("spot_pathplan")
        rospy.Rate(10)

        self._ac = SimpleActionClient("move_base", MoveBaseAction)
        self._ac.wait_for_server(rospy.Duration(5))

        # Send goal to move_base
        result = self.send_goal(0.0, 0.0, 0.0, 1.0)
        rospy.loginfo("Result: {}".format(result))

        # Send goal to move_base
        result = self.send_goal(1.0, 1.0, 0.0, 1.0)
        rospy.loginfo("Result: {}".format(result))

        # Go back to the start
        result = self.send_goal(0.0, 0.0, 0.0, 1.0)
        rospy.loginfo("Result: {}".format(result))

        rospy.spin()


if __name__ == "__main__":
    SpotPathPlan().main()
