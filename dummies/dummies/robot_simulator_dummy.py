import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import JointState

# ur3e
# pos_1 ur3e = [-125.81, -103.99, -47.25, -117.90, 90.62, 54.15]
# pos_2 ur3e = [-155.66, -88.66, -64.57, -116.34, 90.96, 24.30]
# pos_3 ur3e = [-192.49, -105.69, -44.55, -119.78, 90.09, -12.49]
pos_0 = [0.0, -1.5707, 0.0, 0.0, 0.0, 0.0]
pos_1 = [-2.19579873, -1.81496789, -0.82466807, -2.0577432, 1.5816174, 0.94509579]
pos_2 = [-2.71677951, -1.5474089, -1.1269591, -2.03051605, 1.5875515, 0.424115]
pos_3 = [-3.35714082, -1.84202049, -0.78155844, -2.08863552, 1.5692255, 6.0667645]

POSITION_0 = [-2.19579873, -1.81496789, -0.82466807, -2.0577432, 1.5816174, 0.94509579]
POSITION_1 = [-2.71677951, -1.5474089, -1.1269591, -2.03051605, 1.5875515, 0.424115]
POSITION_2 = [-3.35714082, -1.84202049, -0.78155844, -2.08863552, 1.5692255, 0.55555]
POSITION_3 = [0.0, -1.5707, 0.0, 0.0, 0.0, 0.0]

# POSITION_0 = [1.57, 1.57, 1.57, 1.57, 1.57, 1.57]
# POSITION_1 = [-1.57, -1.57, -1.57, -1.57, -1.57, -1.57]
# POSITION_2 = [3.14, 3.14, 3.14, 3.14, 3.14, 3.14]
# POSITION_3 = [-3.14, -3.14, -3.14, -3.14, -3.14, -3.14]

SPEED_0 = 0.5
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
            JointState, "ref_joint_states", 10
        )

        time.sleep(5)

        for i in range(4):
            ref_pos = JointState()
            ref_pos.position = eval("POSITION_{i}".format(i=i))
            ref_pos.name = [self.name + "_" + x for x in self.joint_names]
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
