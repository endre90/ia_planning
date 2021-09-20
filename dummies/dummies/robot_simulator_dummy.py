import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import JointState


POSITION_0 = [1.57, 1.57, 1.57, 1.57, 1.57, 1.57]
POSITION_1 = [-1.57, -1.57, -1.57, -1.57, -1.57, -1.57]
POSITION_2 = [3.14, 3.14, 3.14, 3.14, 3.14, 3.14]
POSITION_3 = [-3.14, -3.14, -3.14, -3.14, -3.14, -3.14]

SPEED_0 = 1.0
SPEED_1 = 0.15
SPEED_2 = 0.8
SPEED_3 = 0.5


class RobotSimulatorDummy(Node):
    def __init__(self):
        super().__init__("robot_simulator_dummy")

        self.name = self.declare_parameter("name", value="robot").value
        self.joint_names = self.declare_parameter(
            "joint_names", value=["j0", "j1", "j2", "j3", "j4", "j5"]
        ).value
        self.initial_position = self.declare_parameter(
            "initial_position", value=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        ).value

        self.publisher = self.create_publisher(
            JointState, "{name}_ref_joint_states".format(name=self.name), 10
        )

        time.sleep(2)

        for i in range(4):
            ref_pos = JointState()
            ref_pos.position = eval("POSITION_{i}".format(i=i))
            ref_pos.name = self.joint_names
            ref_pos.velocity = [
                eval("SPEED_{i}".format(i=i)) for x in range(len(POSITION_0))
            ]

            self.publisher.publish(ref_pos)

            time.sleep(5)


def main(args=None):
    rclpy.init(args=args)

    rs_dummy = RobotSimulatorDummy()
    rclpy.spin(rs_dummy)

    rs_dummy.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
