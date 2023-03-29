import pickle
import typing
import numpy as np
from fiducial import Fiducial
from spot_msgs.msg import WorldObjectArray, WorldObject


def read_fiducial_pickle(path: str):
    fiducials = pickle.load(open(path, "rb"))
    tag_ids = set()

    for tag_id, fiducial in fiducials.items():
        print(f"tag_id: {tag_id}, occurences: {len(fiducial)}")
        tag_ids.add(tag_id)
        fiducial_data: typing.List[Fiducial] = fiducial
        x = [i.filtered_fiducial_pose.pose.position.x for i in fiducial_data]
        y = [i.filtered_fiducial_pose.pose.position.y for i in fiducial_data]
        z = [i.filtered_fiducial_pose.pose.position.z for i in fiducial_data]
        avg_x = sum(x) / len(x)
        avg_y = sum(y) / len(y)
        avg_z = sum(z) / len(z)

        std_dev_x = np.std(x)
        std_dev_y = np.std(y)
        std_dev_z = np.std(z)

        print(
            f"Filtered pose for tag {tag_id}, frame: {fiducial_data[0].filtered_fiducial_pose.header.frame_id}"
        )
        print(f"avg_x: {avg_x:.2f}, avg_y: {avg_y:.2f}, avg_z: {avg_z:.2f}")
        print(
            f"std_dev_x: {std_dev_x:.2f}, std_dev_y: {std_dev_y:.2f}, std_dev_z: {std_dev_z:.2f}\n"
        )

        x = [i.fiducial_pose.pose.position.x for i in fiducial_data]
        y = [i.fiducial_pose.pose.position.y for i in fiducial_data]
        z = [i.fiducial_pose.pose.position.z for i in fiducial_data]
        avg_x = sum(x) / len(x)
        avg_y = sum(y) / len(y)
        avg_z = sum(z) / len(z)
        std_dev_x = np.std(x)
        std_dev_y = np.std(y)
        std_dev_z = np.std(z)
        print(
            f"Unfiltered pose for tag {tag_id}, frame: {fiducial_data[0].fiducial_pose.header.frame_id}"
        )

    print(f"Total tags seen: {len(tag_ids)}")
    print(tag_ids)


def read_waypoint_fiducial_pickle(path: str):
    with open(path, "rb") as f:
        fiducials: typing.Dict[str, typing.List[typing.List["Fiducial"]]] = pickle.load(
            f
        )
        # For each waypoint
        for waypoint, fiducial in fiducials.items():
            fiducial_data = fiducial[0]
            tag_ids_seen = set()

            # For each fiducial seen at that waypoint
            for fiducial_data in f:
                tag_ids_seen.add(str(fiducial_data.tag_id))
            print(
                f"Waypoint: {waypoint}, number of fiducials seen: {len(fiducial)}, fiducial: {', '.join(tag_ids_seen)}"
            )
        print(f"Total waypoints: {len(fiducials)}")


if __name__ == "__main__":
    path = "dir_path_here"
    read_waypoint_fiducial_pickle(f"{path}/waypoint_fiducial.pickle")
    read_fiducial_pickle(f"{path}/fiducials_seen.pickle")
