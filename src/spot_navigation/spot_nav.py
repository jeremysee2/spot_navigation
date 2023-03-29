#!/usr/bin/env python3
import typing
import pickle
import time

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from rosservice import get_service_class_by_name

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from spot_msgs.msg import WorldObjectArray, WorldObject
from spot_msgs.msg import AprilTagProperties
from spot_msgs.msg import FrameTreeSnapshot, ParentEdge
from spot_msgs.msg import NavigateInitAction, NavigateInitGoal
from spot_msgs.msg import NavigateToAction, NavigateToGoal
from spot_msgs.srv import ListGraphResponse
from std_srvs.srv import TriggerRequest

from fiducial import Fiducial


class SpotNav:
    def __init__(self):
        self.world_objects = None
        self.fiducials_seen: typing.Dict[int, typing.List["Fiducial"]] = {}
        self.waypoint_fiducial: typing.Dict[str, typing.List["Fiducial"]] = {}

    def initialize_subscribers(self):
        """Initialize ROS subscribers"""
        # Create a subscriber for the /spot/world_objects topic for WorldObjectArray messages
        self.world_objects_sub = rospy.Subscriber(
            "/spot/world_objects", WorldObjectArray, self.world_objects_callback
        )

    def initialize_publishers(self):
        """Initialize ROS publishers"""
        self.reached_waypoint_pub = rospy.Publisher(
            "/spot/nav/reached_waypoint", String, queue_size=1
        )

    def initialize_action_clients(self):
        """Initialize ROS action clients"""
        # Create an action client for the /spot/navigate_to action
        self.navigate_to_client = actionlib.SimpleActionClient(
            "/spot/navigate_to", NavigateToAction
        )

        # Create an action client for the /spot/navigate_init action
        self.navigate_init_client = actionlib.SimpleActionClient(
            "/spot/navigate_init", NavigateInitAction
        )

    def world_objects_callback(self, msg: WorldObjectArray):
        # Save the message to a class variable tracking the fiducials in the message
        self.world_objects: typing.List[WorldObject] = msg.world_objects

        # Iterate through the fiducials in the message, append the x,y,z coordinates to a dictionary
        for world_object in self.world_objects:
            april_tag: AprilTagProperties = world_object.apriltag_properties
            latest_snapshot: FrameTreeSnapshot = world_object.frame_tree_snapshot

            # Check if april_tag is None
            if april_tag is None:
                continue

            # Create the FrameTreeSnapshot as a dictionary
            frame_tree_snapshot: typing.Dict[str, PoseStamped] = {}
            for child, parent_edge in zip(
                latest_snapshot.child_edges, latest_snapshot.parent_edges
            ):
                parent_edge_transform = PoseStamped()
                parent_edge_transform.header.stamp = world_object.acquisition_time
                parent_edge_transform.header.frame_id = parent_edge.parent_frame_name
                parent_edge_transform.pose = parent_edge.parent_tform_child

                frame_tree_snapshot[child] = parent_edge_transform

            april_tag_pose = frame_tree_snapshot[f"fiducial_{april_tag.tag_id}"]
            april_tag_pose_filtered = frame_tree_snapshot[
                f"filtered_fiducial_{april_tag.tag_id}"
            ]

            # Build the april_tag into the Fiducial class
            fiducial = Fiducial(
                tag_id=april_tag.tag_id,
                dim_x=april_tag.x,
                dim_y=april_tag.y,
                fiducial_pose=april_tag_pose,
                filtered_fiducial_pose=april_tag_pose_filtered,
                pose_covariance=april_tag.detection_covariance,
                pose_covariance_frame=april_tag.detection_covariance_reference_frame,
            )

            # Save the fiducial to the class variable
            if fiducial.tag_id in self.fiducials_seen:
                self.fiducials_seen[fiducial.tag_id].append(fiducial)
            else:
                self.fiducials_seen[fiducial.tag_id] = [fiducial]

            rospy.logdebug(
                f"fiducial: {fiducial.tag_id}\n position_x: {fiducial.fiducial_pose.pose.position.x:.2f}\n filtered_position_x: {fiducial.filtered_fiducial_pose.pose.position.x:.2f}"
            )

    def call_service(self, service_name: str, *args, **kwargs):
        """Call a service and wait for it to be available"""
        try:
            rospy.wait_for_service(service_name)
            service_type = get_service_class_by_name(service_name)
            proxy = rospy.ServiceProxy(service_name, service_type)
            return proxy(*args, **kwargs)

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

    def walk_current_graph(self):
        """Walk the current graph in GraphNav"""
        rospy.loginfo("Walking the current graph")

        # Call the ListGraph service
        list_graph: ListGraphResponse = self.call_service("/spot/list_graph")
        waypoints = list_graph.waypoint_ids

        # Call the /spot/navigate_init action
        navigate_init_goal = NavigateInitGoal(
            upload_path="",
            initial_localization_fiducial=True,
            initial_localization_waypoint="mm",
        )
        self.navigate_init_client.send_goal(navigate_init_goal)
        self.navigate_init_client.wait_for_result()

        # Check if the action succeeded
        if self.navigate_init_client.get_state() == GoalStatus.SUCCEEDED:
            rospy.loginfo("NavigateInit action succeeded")

        for waypoint in waypoints:
            # Call the /spot/navigate_to action
            navigate_to_goal = NavigateToGoal(navigate_to=waypoint)
            self.navigate_to_client.send_goal(navigate_to_goal)
            self.navigate_to_client.wait_for_result()

            # Check if the action succeeded
            if self.navigate_to_client.get_state() == GoalStatus.SUCCEEDED:
                rospy.loginfo(f"NavigateTo {waypoint} action succeeded")
                latest_world_objects_msg = rospy.wait_for_message(
                    "/spot/world_objects", WorldObjectArray
                )
                latest_fiducial = self.extract_fiducials(latest_world_objects_msg)
                if waypoint in self.waypoint_fiducial:
                    self.waypoint_fiducial[waypoint].append(latest_fiducial)
                else:
                    self.waypoint_fiducial[waypoint] = [latest_fiducial]

                # Publish to the /spot/nav/reached_waypoint topic
                self.reached_waypoint_pub.publish(waypoint)
                time.sleep(0.5)

    def extract_fiducials(self, msg: WorldObjectArray) -> typing.List["Fiducial"]:
        self.world_objects: typing.List[WorldObject] = msg.world_objects
        fiducial_list = []

        # Iterate through the fiducials in the message, append the x,y,z coordinates to a dictionary
        for world_object in self.world_objects:
            april_tag: AprilTagProperties = world_object.apriltag_properties
            latest_snapshot: FrameTreeSnapshot = world_object.frame_tree_snapshot

            # Check if april_tag is None
            if april_tag is None:
                continue

            # Create the FrameTreeSnapshot as a dictionary
            frame_tree_snapshot: typing.Dict[str, PoseStamped] = {}
            for child, parent_edge in zip(
                latest_snapshot.child_edges, latest_snapshot.parent_edges
            ):
                parent_edge_transform = PoseStamped()
                parent_edge_transform.header.stamp = world_object.acquisition_time
                parent_edge_transform.header.frame_id = parent_edge.parent_frame_name
                parent_edge_transform.pose = parent_edge.parent_tform_child

                frame_tree_snapshot[child] = parent_edge_transform

            # Use tf2 to get the april_tag pose in the body frame
            april_tag_pose = frame_tree_snapshot[f"fiducial_{april_tag.tag_id}"]
            april_tag_pose_filtered = frame_tree_snapshot[
                f"filtered_fiducial_{april_tag.tag_id}"
            ]

            # Build the april_tag into the Fiducial class
            fiducial = Fiducial(
                tag_id=april_tag.tag_id,
                dim_x=april_tag.x,
                dim_y=april_tag.y,
                fiducial_pose=april_tag_pose,
                filtered_fiducial_pose=april_tag_pose_filtered,
                pose_covariance=april_tag.detection_covariance,
                pose_covariance_frame=april_tag.detection_covariance_reference_frame,
            )

            # Append the fiducial to the list
            fiducial_list.append(fiducial)

        return fiducial_list

    def startup(self):
        rospy.loginfo("SpotNav robot starting up")

        # Call the /spot/claim, /spot/power_on, /spot/stand service
        self.call_service("/spot/claim", TriggerRequest())
        self.call_service("/spot/power_on", TriggerRequest())

    def shutdown(self):
        rospy.loginfo("SpotNav node shutting down")

        # Save the fiducials to a pickle file
        with open(f"fiducials_seen.pickle_{time.time()}", "wb") as f:
            pickle.dump(self.fiducials_seen, f)
        with open(f"waypoint_fiducial.pickle_{time.time()}", "wb") as f:
            pickle.dump(self.waypoint_fiducial, f)

        # Call the /spot/sit, /spot/power_off, /spot/release service
        self.call_service("/spot/sit", TriggerRequest())
        self.call_service("/spot/power_off", TriggerRequest())
        self.call_service("/spot/release", TriggerRequest())

    def main(self):
        rospy.init_node("spot_nav", anonymous=True)

        self.rates = rospy.get_param("~rates", {})
        if "loop_frequency" in self.rates:
            loop_rate = self.rates["loop_frequency"]
        else:
            loop_rate = 50

        # Initialize the node
        rate = rospy.Rate(loop_rate)
        rospy.loginfo("SpotNav node started")

        self.initialize_subscribers()
        self.initialize_publishers()
        self.initialize_action_clients()

        rospy.on_shutdown(self.shutdown)

        # Walk the current graph
        self.startup()
        self.walk_current_graph()

        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == "__main__":
    spot_nav = SpotNav()
    spot_nav.main()
