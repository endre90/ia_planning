import rclpy

from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time

r1_poses = {
    "pos1": [-125.81, -103.99, -47.25, -117.90, 90.62, 54.15],
    "pos2": [-155.66, -88.66, -64.57, -116.34, 90.96, 24.30],
    "pos3": [-192.49, -105.69, -44.55, -119.78, 90.09, -12.49],
    "home": [0.0, -90, 0.0, 0.0, 0.0, 0.0],
}

r2_poses = {
    "pos1": [-7.55, -95.36, -97.31, -77.34, 90.09, -7.55],
    "pos2": [15.38, -90.0, -102.96, -77.02, 90.09, 15.38],
    "pos3": [36.07, -95.29, -97.41, -77.25, 90.07, 36.07],
    "home": [0.0, -90, 0.0, 0.0, 0.0, 0.0],
}

class RobotController(Node):
    def __init__(self):
        super().__init__("robot_controller")

        self.ref_joint_msg = JointState()
        self.act_pos_msg = String()

        self.name = self.declare_parameter("name", value="robot").value

        if self.name == "r1":
            self.poses = {
                y: [round(x * 3.1415926 / 180, 5) for x in r1_poses[y]]
                for y in r1_poses
            }
        elif self.name == "r2":
            self.poses = {
                y: [round(x * 3.1415926 / 180, 5) for x in r2_poses[y]]
                for y in r2_poses
            }
        else:
            self.poses = {"home": [0.0, -1.5707, 0.0, 0.0, 0.0, 0.0]}

        self.ref_pos = "home"
        self.act_pos = "unknown"

        self.state_publisher_ = self.create_publisher(
            String,
            "act_pos",
            10,
        )

        self.state_timer_period = 0.1
        self.state_publisher_timer = self.create_timer(
            self.state_timer_period, self.state_ticker
        )

        self.ref_joint_state_publisher_ = self.create_publisher(
            JointState,
            "ref_joint_states",
            10,
        )

        self.ref_joint_state_timer_period = 1.0
        self.ref_joint_state_publisher_timer = self.create_timer(
            self.ref_joint_state_timer_period, self.ref_joint_state_ticker
        )

        self.command_subscriber = self.create_subscription(
            String,
            "ref_pos",
            self.command_subscriber_callback,
            10,
        )

        self.joint_state_subscriber = self.create_subscription(
            JointState,
            "joint_states",
            self.joint_states_subscriber_callback,
            10,
        )

    def command_subscriber_callback(self, data):
        self.ref_pos = data.data

    def joint_states_subscriber_callback(self, data):
        for pose in self.poses:
            if self.poses[pose] == list(data.position):
                self.act_pos = pose
                break
            else:
                self.act_pos = "unknown"

    def state_ticker(self):
        self.act_pos_msg.data = self.act_pos
        self.state_publisher_.publish(self.act_pos_msg)

    def ref_joint_state_ticker(self):

        time = Time()
        now = self.get_clock().now().seconds_nanoseconds()
        time.sec = now[0]
        time.nanosec = now[1]
        self.ref_joint_msg.header.stamp = time
        self.ref_joint_msg.position = self.poses[self.ref_pos]
        self.ref_joint_msg.velocity = [0.75 for x in self.poses[self.ref_pos]]
        self.ref_joint_state_publisher_.publish(self.ref_joint_msg)


def main(args=None):
    rclpy.init(args=args)
    robot_simulator = RobotController()

    rclpy.spin(robot_simulator)
    robot_simulator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
