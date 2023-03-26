import pickle
import typing
from fiducial import Fiducial


def read_fiducial_pickle():
    fiducials = pickle.load(open("fiducials_seen.pickle", "rb"))

    for tag_id, fiducial in fiducials.items():
        print(f"tag_id: {tag_id}, occurences: {len(fiducial)}")
        fiducial_data: typing.List[Fiducial] = fiducial
        x = [i.filtered_fiducial_pose.pose.position.x for i in fiducial_data]
        y = [i.filtered_fiducial_pose.pose.position.y for i in fiducial_data]
        z = [i.filtered_fiducial_pose.pose.position.z for i in fiducial_data]
        avg_x = sum(x) / len(x)
        avg_y = sum(y) / len(y)
        avg_z = sum(z) / len(z)
        print(f"Filtered pose for tag {tag_id}")
        print(f"avg_x: {avg_x:.2f}, avg_y: {avg_y:.2f}, avg_z: {avg_z:.2f}")

        x = [i.fiducial_pose.pose.position.x for i in fiducial_data]
        y = [i.fiducial_pose.pose.position.y for i in fiducial_data]
        z = [i.fiducial_pose.pose.position.z for i in fiducial_data]
        avg_x = sum(x) / len(x)
        avg_y = sum(y) / len(y)
        avg_z = sum(z) / len(z)
        print(f"Unfiltered pose for tag {tag_id}")
        print(f"avg_x: {avg_x:.2f}, avg_y: {avg_y:.2f}, avg_z: {avg_z:.2f}\n")


if __name__ == "__main__":
    read_fiducial_pickle()
