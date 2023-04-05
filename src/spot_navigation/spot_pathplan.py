#! /usr/bin/env python3
import typing
import rospy
from actionlib import SimpleActionClient

from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import (
    MoveBaseAction,
    MoveBaseGoal,
    MoveBaseResult,
    MoveBaseFeedback,
)
from rosservice import get_service_class_by_name
import rtabmap_ros
from rtabmap_ros.srv import GetMap, GetMapRequest, GetMapResponse
from rtabmap_ros.srv import SetGoal, SetGoalRequest, SetGoalResponse
from rtabmap_ros.srv import ListLabelsRequest, ListLabelsResponse
from rtabmap_ros.srv import SetLabelRequest, SetLabelResponse
from rtabmap_ros.msg import MapData, NodeData


class SpotPathPlan(object):
    def call_service(self, service_name: str, *args, **kwargs):
        """Call a service and wait for it to be available"""
        try:
            rospy.wait_for_service(service_name)
            service_type = get_service_class_by_name(service_name)
            proxy = rospy.ServiceProxy(service_name, service_type)
            return proxy(*args, **kwargs)

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

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

    def map_data_callback(self, msg: MapData) -> typing.Dict[int, PoseStamped]:
        # Extract the graph and poses from the map
        graph = msg.graph
        nodes: typing.List[NodeData] = msg.nodes

        # Extract the list of waypoints from the poses
        waypoints = {}
        for node in nodes:
            waypoints[node.id] = node.pose.position

        # Print the list of waypoints
        rospy.loginfo("List of waypoints: {}".format(waypoints))

        return waypoints

    def get_map_data(self):
        # Get the map data
        req = GetMapRequest()
        req.global_ = True
        req.optimized = True
        req.graph_only = False
        map_data: GetMapResponse = self.call_service("/rtabmap/get_map", req)

        # Extract the graph and poses from the map
        waypoints = self.map_data_callback(map_data)

        # Store the waypoints in the class
        self.waypoints = waypoints

    def set_goal(self, goal_id: int):
        # Set the goal
        goal = SetGoalRequest()
        goal.node_id = goal_id
        resp: SetGoalResponse = self.call_service("/rtabmap/set_goal", goal)

        # Print the result
        path_ids = resp.path_ids
        path_poses = resp.path_poses
        planning_time = resp.planning_time

        rospy.loginfo("Path IDs: {}".format(path_ids))
        rospy.loginfo("Path Poses: {}".format(path_poses))
        rospy.loginfo("Planning Time: {}".format(planning_time))

    def backup_map(self):
        # Call the /rtabmap/backup service
        self.call_service("/rtabmap/backup_database")

    def list_labels(self) -> typing.List[str]:
        # Call the /rtabmap/list_labels service
        resp: ListLabelsResponse = self.call_service("/rtabmap/list_labels")
        labels: typing.List[str] = resp.labels
        return labels

    def set_label(self, node_id: int, label: str):
        # Call the /rtabmap/set_label service
        req = SetLabelRequest()
        req.node_id = node_id
        req.node_label = label
        self.call_service("/rtabmap/set_label", req)

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
    rospy.init_node("waypoint_extractor")
    SpotPathPlan().main()
    rospy.spin()
