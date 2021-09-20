import rclpy
import math
import json

from rclpy.node import Node
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time


class RobotSimulator(Node):
    def __init__(self):
        super().__init__("robot_simulator")

        self.act = JointState()
        self.ref = JointState()

        self.name = self.declare_parameter("name", value="robot").value
        self.joint_names = self.declare_parameter(
            "joint_names", value=["j0", "j1", "j2", "j3", "j4", "j5"]
        ).value
        self.initial_position = self.declare_parameter(
            "initial_position", value=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        ).value

        self.act.position = self.initial_position
        self.ref.position = self.initial_position

        self.act.name = self.joint_names

        self.joint_state_publisher_ = self.create_publisher(
            JointState,
            "{name}_joint_states".format(name=self.name),
            10,
        )

        self.joint_state_subscriber = self.create_subscription(
            JointState,
            "{name}_ref_joint_states".format(name=self.name),
            self.joint_state_subscriber_callback,
            10,
        )

        self.joint_state_timer_period = 0.01

        self.joint_state_publisher_timer = self.create_timer(
            self.joint_state_timer_period, self.joint_state_ticker
        )

    def joint_state_subscriber_callback(self, data):
        self.ref = data

    def joint_state_ticker(self):

        if not self.act.position == self.ref.position:

            ssc = [
                abs(self.ref.position[i] - self.act.position[i])
                for i in range(len(self.ref.position))
            ]

            sync_max = max(ssc)
            sync_max_factor = 1

            if sync_max != 0:
                sync_max_factor = 1 / sync_max

            ssc = list(
                map(
                    lambda i: ssc[i] * sync_max_factor,
                    range(len(self.ref.position)),
                )
            )

            for i in range(len(self.ref.position)):
                rad_sec = self.ref.velocity[i] * 100.0 * math.pi / 180
                step = rad_sec * self.joint_state_timer_period

                if (self.ref.position[i] < (self.act.position[i] - step)) and ssc[
                    i
                ] > 0.1:
                    self.act.position[i] = round(
                        self.act.position[i] - (step * ssc[i]), 5
                    )
                elif (self.ref.position[i] > (self.act.position[i] + step)) and ssc[
                    i
                ] > 0.1:
                    self.act.position[i] = round(
                        self.act.position[i] + (step * ssc[i]), 5
                    )
                else:
                    self.act.position[i] = self.ref.position[i]

        self.joint_state_publisher_.publish(self.act)


def main(args=None):
    rclpy.init(args=args)
    robot_simulator = RobotSimulator()

    rclpy.spin(robot_simulator)
    robot_simulator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
