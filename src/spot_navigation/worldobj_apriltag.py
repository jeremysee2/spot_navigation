#!/usr/bin/env python3
import rospy
import typing

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from apriltag_ros.msg import AprilTagDetectionArray, AprilTagDetection
from spot_msgs.msg import (
    WorldObjectArray,
    WorldObject,
    AprilTagProperties,
    FrameTreeSnapshot,
)


class WorldObjToAprilTag:
    def extract_frame_tree(self, world_obj: WorldObject) -> typing.List[PoseStamped]:
        # Create the FrameTreeSnapshot as a dictionary
        latest_snapshot: FrameTreeSnapshot = world_obj.frame_tree_snapshot
        apriltag: AprilTagProperties = world_obj.apriltag_properties

        frame_tree_snapshot: typing.Dict[str, PoseStamped] = {}
        for child, parent_edge in zip(
            latest_snapshot.child_edges, latest_snapshot.parent_edges
        ):
            parent_edge_transform = PoseStamped()
            parent_edge_transform.header.stamp = world_obj.acquisition_time
            parent_edge_transform.header.frame_id = parent_edge.parent_frame_name
            parent_edge_transform.pose = parent_edge.parent_tform_child

            frame_tree_snapshot[child] = parent_edge_transform

        april_tag_pose = frame_tree_snapshot[f"fiducial_{apriltag.tag_id}"]
        april_tag_pose_filtered = frame_tree_snapshot[
            f"filtered_fiducial_{apriltag.tag_id}"
        ]
        return april_tag_pose, april_tag_pose_filtered

    def convert(self, world_obj_array: WorldObjectArray) -> AprilTagDetectionArray:
        result = AprilTagDetectionArray(header=None, detections=[])
        for world_obj in world_obj_array.world_objects:
            if world_obj.apriltag_properties is None:
                return

            apriltag: AprilTagProperties = world_obj.apriltag_properties
            apriltag_msg: AprilTagDetection = AprilTagDetection()

            apriltag_msg.id = [apriltag.tag_id]
            apriltag_msg.size = [(apriltag.x + apriltag.y) / 2]

            # Set pose
            pose = PoseWithCovarianceStamped()
            apriltag_pose, apriltag_pose_filtered = self.extract_frame_tree(world_obj)
            pose.header = apriltag_pose.header
            result.header = apriltag_pose.header
            pose.pose.pose = apriltag_pose.pose
            # Set covariance matrix
            pose.pose.covariance = apriltag.detection_covariance.covariance

            apriltag_msg.pose = pose

            # Add to result
            result.detections.append(apriltag_msg)

        return result

    def initialize_subscriber_publisher(self):
        self.world_obj_sub = rospy.Subscriber(
            "/spot/world_objects", WorldObjectArray, self.world_obj_callback
        )
        self.apriltag_pub = rospy.Publisher(
            "/tag_detections", AprilTagDetectionArray, queue_size=10
        )

    def world_obj_callback(self, world_obj_array: WorldObjectArray):
        self.apriltag_pub.publish(self.convert(world_obj_array))

    def shutdown(self):
        rospy.loginfo("Shutting down worldobj_apriltag node")

    def main(self):
        rospy.init_node("worldobj_apriltag", anonymous=True)
        rospy.on_shutdown(self.shutdown)
        self.initialize_subscriber_publisher()
        rospy.spin()


if __name__ == "__main__":
    app = WorldObjToAprilTag()
    app.main()
