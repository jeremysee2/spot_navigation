#!/usr/bin/env python3
import typing

import tf2_ros, tf2_geometry_msgs, tf2_sensor_msgs
import rospy

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2


class SpotTransforms:
    """
    Class for transforming:
    - PointCloud2 messages from odom to body frame
    """

    def __init__(self) -> None:
        pass

    def initialize_publishers(self) -> None:
        """Initialize ROS publishers"""
        # Create a publisher for the /spot/nav/lidar/points topic for PointCloud2 messages
        self.pcl_pub = rospy.Publisher(
            "/spot/nav/lidar/points", PointCloud2, queue_size=10
        )

    def initialize_subscribers(self) -> None:
        """Initialize ROS subscribers"""
        # Subscribe to /spot/lidar/points topic for PointCloud2 messages
        self.lidar_points_sub = rospy.Subscriber(
            "/spot/lidar/points", PointCloud2, self.lidar_points_callback
        )

    def initialize_tf2(self):
        """Initialize tf2_ros.Buffer and tf2_ros.TransformListener objects"""
        # Create a tf2_ros.Buffer object
        self.tf_buffer = tf2_ros.Buffer()

        # Create a tf2_ros.TransformListener object
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def lidar_points_callback(self, msg: PointCloud2) -> None:
        """Converts PointCloud2 message from odom to body frame and publishes it to /spot/nav/lidar/points"""
        pcl_msg = self._transform_pcl_to_body_frame(msg)

        # Publish the PointCloud2 message to /spot/nav/lidar/points
        self.pcl_pub.publish(pcl_msg)

    def _transform_pcl_to_body_frame(self, pcl: PointCloud2) -> PointCloud2:
        if pcl.header.frame_id == "body":
            return pcl

        if self.tf_buffer.can_transform("body", pcl.header.frame_id, pcl.header.stamp):
            body_to_fixed = self.tf_buffer.lookup_transform(
                "body",
                pcl.header.frame_id,
                pcl.header.stamp,
            )
        else:
            rospy.logwarn(
                f"Cannot transform from {pcl.header.frame_id} to body frame. Returning original pcl."
            )
            return pcl

        pcl_in_body = tf2_sensor_msgs.do_transform_cloud(pcl, body_to_fixed)
        pcl_in_body.header.frame_id = "body"
        rospy.logdebug(
            f"Transformed PointCloud2 message from {pcl.header.frame_id} to {pcl_in_body.header.frame_id} frame."
        )

        return pcl_in_body

    def shutdown(self) -> None:
        """Shut down ROS node when Ctl+C is pressed"""
        rospy.loginfo("Shutting down ROS Transforms node...")

    def main(self) -> None:
        """Main function"""
        # Initialize ROS node
        rospy.init_node("spot_transforms", anonymous=True)

        # Initialize ROS publishers
        self.initialize_publishers()

        # Initialize ROS subscribers
        self.initialize_subscribers()

        # Initialize tf2_ros.Buffer and tf2_ros.TransformListener objects
        self.initialize_tf2()

        # Start the ROS node
        rospy.on_shutdown(self.shutdown)
        rospy.spin()


if __name__ == "__main__":
    spot_transforms = SpotTransforms()
    spot_transforms.main()
