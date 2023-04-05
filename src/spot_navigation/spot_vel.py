import rospy
import time
from actionlib_msgs.msg import GoalStatus
from rosservice import get_service_class_by_name

from geometry_msgs.msg import Twist
from std_srvs.srv import TriggerRequest


class SpotVel:
    def __init__(self) -> None:
        # Set up a cmd_vel publisher
        self._cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

    def call_service(self, service_name: str, *args, **kwargs):
        """Call a service and wait for it to be available"""
        try:
            rospy.wait_for_service(service_name)
            service_type = get_service_class_by_name(service_name)
            proxy = rospy.ServiceProxy(service_name, service_type)
            return proxy(*args, **kwargs)

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

    def send_vel(self, linear_x, angular_z):
        # Create a Twist message and add linear x and angular z values
        rospy.loginfo(f"Sending linear x: {linear_x} and angular z: {angular_z}")
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z

        # Publish the Twist message and sleep 1 cycle
        self._cmd_vel_pub.publish(twist)
        rospy.sleep(1)

    def start_robot(self):
        rospy.loginfo("SpotNav robot starting up")

        # Call the /spot/claim, /spot/power_on, /spot/stand service
        self.call_service("/spot/claim", TriggerRequest())
        self.call_service("/spot/power_on", TriggerRequest())
        self.call_service("/spot/stand", TriggerRequest())

    def stop_robot(self):
        # Create a Twist message and add linear x and angular z values
        rospy.loginfo("Stopping the robot")
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0

        # Publish the Twist message and sleep 1 cycle
        self._cmd_vel_pub.publish(twist)
        rospy.sleep(1)
        self.call_service("/spot/sit", TriggerRequest())

    def shutdown(self):
        # Stop the robot
        self.stop_robot()

        # Call the /spot/power_off, /spot/release service
        self.call_service("/spot/sit", TriggerRequest())
        time.sleep(3)
        self.call_service("/spot/power_off", TriggerRequest())
        self.call_service("/spot/release", TriggerRequest())

    def main(self):
        # Initialize the node
        rospy.init_node("spot_vel")
        rospy.on_shutdown(self.shutdown)

        # Start the robot
        self.start_robot()
        time.sleep(5)

        # Send the robot forward at 1.0 m/s
        self.send_vel(0.5, 0.0)
        rospy.sleep(3)

        # Send the robot forward at 1.0 m/s
        self.send_vel(-0.5, 0.0)
        rospy.sleep(3)

        # Send the robot forward at 1.0 m/s
        self.send_vel(0.0, 1.0)
        rospy.sleep(3)

        # Stop the robot
        self.stop_robot()

        rospy.spin()


if __name__ == "__main__":
    SpotVel().main()
