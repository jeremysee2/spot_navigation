import rospy
import pickle

from geometry_msgs.msg import PoseStamped
from spot_msgs.msg import WorldObjectArray


class MockROSPublisher:
    def publish_world_objects(self, world_objects):
        self._world_objects_pub.publish(world_objects)

    def load_world_objects(self, file_path):
        with open(file_path, "rb") as f:
            world_objects = pickle.load(f)
        return world_objects

    def shutdown(self):
        self._world_objects_pub.unregister()

    def main(self):
        rospy.init_node("mock_ros_publisher", anonymous=True)

        rate = rospy.Rate(10)
        rospy.on_shutdown(self.shutdown)

        self._world_objects_pub = rospy.Publisher(
            "/spot/world_objects", WorldObjectArray, queue_size=10
        )

        with open("data/world_objects_msg.pkl", "rb") as f:
            msg_world_objects = pickle.load(f)

        while not rospy.is_shutdown():
            self.publish_world_objects(msg_world_objects)
            rate.sleep()


if __name__ == "__main__":
    mock_ros_publisher = MockROSPublisher()
    mock_ros_publisher.main()
