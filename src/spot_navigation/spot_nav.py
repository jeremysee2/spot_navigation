#!/usr/bin/env python3
import typing
import pickle

import tf2_ros, tf2_geometry_msgs
import rospy

from geometry_msgs.msg import Pose, PoseStamped
from spot_msgs.msg import WorldObjectArray, WorldObject
from spot_msgs.msg import AprilTagProperties
from spot_msgs.msg import FrameTreeSnapshot, ParentEdge

from spot_navigation.fiducial import Fiducial


class SpotNav:
    def __init__(self):
        self.world_objects = None
        self.fiducials_seen: typing.Dict[int, typing.List["Fiducial"]] = {}

    def initialize_subscribers(self):
        """Initialize ROS subscribers"""
        # Create a subscriber for the /spot/world_objects topic for WorldObjectArray messages
        self.world_objects_sub = rospy.Subscriber(
            "/spot/world_objects", WorldObjectArray, self.world_objects_callback
        )

    def world_objects_callback(self, msg: WorldObjectArray):
        rospy.loginfo("world_objects_callback")
        rospy.loginfo(str(msg))

        # Save the message to a class variable tracking the fiducials in the message
        self.world_objects: typing.List[WorldObject] = msg.world_objects

        # Iterate through the fiducials in the message, append the x,y,z coordinates to a dictionary
        for world_object in self.world_objects:
            april_tag: AprilTagProperties = world_object.apriltag_properties
            latest_snapshot: FrameTreeSnapshot = world_object.frame_tree_snapshot

            # Check if april_tag is None
            if april_tag is None:
                continue
            rospy.loginfo(
                f"Found tag {april_tag.tag_id} at {world_object.frame_tree_snapshot.child_edges[0].parent_tform_child.position}"
            )

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
            april_tag_pose = self._transform_pose_to_body_frame(
                frame_tree_snapshot[f"fiducial_{april_tag.tag_id}"]
            )
            april_tag_pose_filtered = self._transform_pose_to_body_frame(
                frame_tree_snapshot[f"filtered_fiducial_{april_tag.tag_id}"]
            )

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

    def initialize_tf2(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def _transform_pose_to_body_frame(self, pose: PoseStamped) -> PoseStamped:
        if pose.header.frame_id == "body":
            return pose

        if self.tf_buffer.can_transform("body", pose.header.frame_id, rospy.Time()):
            body_to_fixed = self.tf_buffer.lookup_transform(
                "body",
                pose.header.frame_id,
                rospy.Time(),  # TODO: replace with pose.header.stamp
            )
        else:
            rospy.logwarn(
                f"Cannot transform from {pose.header.frame_id} to body frame. Returning original pose."
            )
            return pose

        pose_in_body = tf2_geometry_msgs.do_transform_pose(pose, body_to_fixed)
        pose_in_body.header.frame_id = "body"

        return pose_in_body

    def shutdown(self):
        rospy.loginfo("SpotNav node shutting down")

        # Save the fiducials to a pickle file
        with open("fiducials_seen.pickle", "wb") as f:
            pickle.dump(self.fiducials_seen, f)

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
        self.initialize_tf2()

        rospy.on_shutdown(self.shutdown)

        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == "__main__":
    spot_nav = SpotNav()
    spot_nav.main()
