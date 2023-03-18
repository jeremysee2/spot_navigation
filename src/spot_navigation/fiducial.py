import typing
import numpy as np

from geometry_msgs.msg import PoseStamped


class Fiducial:
    """
    Fiducial class that stores the fiducial information in standard ROS geometry_msgs/Pose format
    """

    def __init__(
        self,
        tag_id: int,
        dim_x: float,
        dim_y: float,
        fiducial_pose: PoseStamped,
        filtered_fiducial_pose: PoseStamped,
        pose_covariance: typing.Optional[typing.List[float]] = None,
        pose_covariance_frame: typing.Optional[str] = None,
    ):
        # Save variables to class
        self.tag_id = tag_id
        self.dim_x = dim_x
        self.dim_y = dim_y
        self.fiducial_pose = fiducial_pose
        self.filtered_fiducial_pose = filtered_fiducial_pose
        if pose_covariance and pose_covariance_frame:
            self.pose_covariance = pose_covariance  # Row-major
            self.pose_covariance_frame = pose_covariance_frame

    def distance_to_fiducial(self, other: "Fiducial"):
        """Returns the distance between two fiducials"""
        return np.linalg.norm(
            np.array(
                [
                    self.fiducial_pose.pose.position.x,
                    self.fiducial_pose.pose.position.y,
                    self.fiducial_pose.pose.position.z,
                ]
            )
            - np.array(
                [
                    other.fiducial_pose.pose.position.x,
                    other.fiducial_pose.pose.position.y,
                    other.fiducial_pose.pose.position.z,
                ]
            )
        )


if __name__ == "__main__":
    pass
