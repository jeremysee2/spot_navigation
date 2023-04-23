"""ROS node to transform all data to the /body frame
"""

import rospy
import sys
import numpy as np
import pickle
import yaml

from std_msgs.msg import Header
from geometry_msgs.msg import Vector3, Quaternion
from geometry_msgs.msg import PoseWithCovariance, PoseStamped
from geometry_msgs.msg import TwistWithCovarianceStamped
from geometry_msgs.msg import TransformStamped, Transform

import tf2_ros
import tf2_geometry_msgs
from tf import transformations as ts

# Add ROS packages to the path
sys.path.append('/com.docker.devenvironments.code/catkin_ws/devel/lib/python3/dist-packages')
sys.path.append('/opt/ros/noetic/lib/python3/dist-packages/')

import fiducial

class FiducialTF:
    """ROS node to transform all data to the /body frame
    """

    def __init__(self):
        self.sensors_static_transforms = []
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.sensors_static_transform_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.transform_broadcaster = tf2_ros.TransformBroadcaster()

    def invertTf(self, x, y, z, qx, qy, qz, qw):
        transform = ts.concatenate_matrices(ts.translation_matrix([x,y,z]), ts.quaternion_matrix([qx,qy,qz,qw]))
        inversed_transform = ts.inverse_matrix(transform)
        return ts.translation_from_matrix(inversed_transform), ts.quaternion_from_matrix(inversed_transform)

    def populateStaticTf(self, x, y, z, qx, qy, qz, qw, frame_id, child_frame_id, header_seq, header_stamp_s, header_stamp_ns):
        tf_time = rospy.Time(secs=header_stamp_s, nsecs=header_stamp_ns)

        static_tf = TransformStamped(
            header=Header(
                seq=header_seq,
                frame_id=frame_id,
                stamp=tf_time
            ),
            child_frame_id=child_frame_id,
            transform=Transform(
                translation=Vector3(
                    x=x,
                    y=y,
                    z=z
                ),
                rotation=Quaternion(
                    x=qx,
                    y=qy,
                    z=qz,
                    w=qw
                )
            )
        )
        self.sensors_static_transforms.append(static_tf)

    def broadcastStaticTf(self):
        self.transform_broadcaster.sendTransform(
            self.sensors_static_transforms
        )

    def _transform_pose_to_body_frame(self, pose: PoseStamped) -> PoseStamped:
        """
        Transform a pose to the body frame

        Args:
            pose: PoseStamped to transform

        Raises: tf2_ros.LookupException if the transform lookup fails

        Returns: Transformed pose in body frame if given pose is not in the body frame, or the original pose if it is
        in the body frame
        """
        if pose.header.frame_id == "body":
            return pose

        body_to_fixed = self.tf_buffer.lookup_transform(
            "body", pose.header.frame_id, rospy.Time()
        )

        pose_in_body = tf2_geometry_msgs.do_transform_pose(pose, body_to_fixed)
        pose_in_body.header.frame_id = "body"

        return pose_in_body
    
    def read_pickle(self, indoor = True):
        """Reads the pickle file and returns the data
        """
        test_data_path = '/com.docker.devenvironments.code/test_data/'

        # Set the path to the data for all 3 walks
        fid_path_indoor = [
            test_data_path + 'walk2/fid-walk2.pickle',
            test_data_path + 'walk3/fid-walk3.pickle',
            test_data_path + 'walk4/fid-walk4.pickle'
        ]

        way_fid_path_indoor = [
            test_data_path + '27Mar/walk2/way-walk2.pickle',
            test_data_path + '27Mar/walk3/way-walk3.pickle',
            test_data_path + '27Mar/walk4/way-walk4.pickle'
        ]

        fid_path_outdoor = [
            test_data_path + '28Mar/fid1.pickle',
            test_data_path + '28Mar/fid2.pickle',
            test_data_path + '28Mar/fid3.pickle'
        ]

        way_fid_path_outdoor = [
            test_data_path + '28Mar/way1.pickle',
            test_data_path + '28Mar/way2.pickle',
            test_data_path + '28Mar/way3.pickle'
        ]

        if indoor:
            fid_path = fid_path_indoor
            way_fid_path = way_fid_path_indoor
            output_path = test_data_path + "variation_indoor.txt"
        else:
            fid_path = fid_path_outdoor
            way_fid_path = way_fid_path_outdoor
            output_path = test_data_path + "variation_outdoor.txt"

        # Load the data
        way_fid_data = []
        for p in way_fid_path:
            with open(p, 'rb') as f:
                way_fid_data.append(pickle.load(f))

        waypoints = list(way_fid_data[0].keys())
        n = len(way_fid_data)

        # For each run
        for i in range(3):
            # For each waypoint
            for waypoint, d in way_fid_data[i].items():
                detections = d[0]
                # For each fiducial
                for fid in detections:
                    # Transform fiducial into body frame
                    fid.fiducial_pose = self._transform_pose_to_body_frame(fid.fiducial_pose)

        # Save way_fid_data as pickle
        if indoor:
            outfile = "indoor_way_fid_data.pickle"
        else:
            outfile = "outdoor_way_fid_data.pickle"

        with open(outfile, 'wb') as f:
            pickle.dump(way_fid_data, f)

if __name__ == '__main__':
    # Init ros node
    rospy.init_node("fiducial_tf")

    # Init transform class
    fid_tf = FiducialTF()
    rospy.sleep(3)

    # Load static transforms from file
    with open("tf_static.yaml", "r") as stream:
        try:
            tf_static_yml = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)

    # Populate static transforms
    for tf in tf_static_yml["transforms"]:
        fid_tf.populateStaticTf(
            x=tf['transform']['translation']['x'],
            y=tf['transform']['translation']['y'],
            z=tf['transform']['translation']['z'],
            qx=tf['transform']['rotation']['x'],
            qy=tf['transform']['rotation']['y'],
            qz=tf['transform']['rotation']['z'],
            qw=tf['transform']['rotation']['w'],
            frame_id=tf['header']['frame_id'],
            child_frame_id=tf['child_frame_id'],
            header_seq=tf['header']['seq'],
            header_stamp_s=tf['header']['stamp']['secs'],
            header_stamp_ns=tf['header']['stamp']['nsecs']
        )

        # Invert the transform and populate the static transforms
        inverted_tf_translation, inverted_tf_quaternion = fid_tf.invertTf(
            x=tf['transform']['translation']['x'],
            y=tf['transform']['translation']['y'],
            z=tf['transform']['translation']['z'],
            qx=tf['transform']['rotation']['x'],
            qy=tf['transform']['rotation']['y'],
            qz=tf['transform']['rotation']['z'],
            qw=tf['transform']['rotation']['w'],
        )

        fid_tf.populateStaticTf(
            x=inverted_tf_translation[0],
            y=inverted_tf_translation[1],
            z=inverted_tf_translation[2],
            qx=inverted_tf_quaternion[0],
            qy=inverted_tf_quaternion[1],
            qz=inverted_tf_quaternion[2],
            qw=inverted_tf_quaternion[3],
            frame_id=tf['child_frame_id'],
            child_frame_id=tf['header']['frame_id'],
            header_seq=tf['header']['seq'],
            header_stamp_s=tf['header']['stamp']['secs'],
            header_stamp_ns=tf['header']['stamp']['nsecs']
        )

    # Broadcast static transforms
    fid_tf.broadcastStaticTf()

    # Read the data
    fid_tf.read_pickle(indoor=False)