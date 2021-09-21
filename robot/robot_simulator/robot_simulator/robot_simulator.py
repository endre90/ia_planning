import rclpy
import math

from rclpy.node import Node
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time

"""
A joint state simulator that is agnostic to the 
number of joints. Should be launched through a 
launch file to configure it. In order to avoid
compiling special messages for this simulator,
the JointState message is used to both set the
ref position through the position field and also 
the velocity scaling through the velocity field.
"""


class RobotSimulator(Node):
    def __init__(self):
        super().__init__("robot_simulator")

        self.act = JointState()
        self.ref = JointState()

        """
        Declaring default values. When the node is launched through a
        launch file, these parameters can be set with different values.
        Look at /bringup/dummies/robot_simulator_test.launch.py.
        """
        self.name = self.declare_parameter("name", value="robot").value
        self.joint_names = self.declare_parameter(
            "joint_names", value=["j0", "j1", "j2", "j3", "j4", "j5"]
        ).value
        self.initial_position = self.declare_parameter(
            "initial_position", value=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        ).value

        """
        As this node simulates the joint positions, we have no idea about
        where the robot is when we start the simulator. So, this sets the
        initial position. 
        """
        self.act.position = self.initial_position
        self.ref.position = self.initial_position

        self.act.name = [self.name + "_" + x for x in self.joint_names]
        # self.act.name = self.joint_names
        """
        This creates a publisher which publishes the actual position.
        """
        self.joint_state_publisher_ = self.create_publisher(
            JointState,
            "joint_states",
            10,
        )

        """
        This creates a subscriber which subscribes to the reference position.
        Every time a message arrives, the callback function is called.
        """
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            "ref_joint_states",
            self.joint_state_subscriber_callback,
            10,
        )

        """
        This creates a thread to call a function at a certain rate (period).
        """
        self.joint_state_timer_period = 0.01
        self.joint_state_publisher_timer = self.create_timer(
            self.joint_state_timer_period, self.joint_state_ticker
        )

    def joint_state_subscriber_callback(self, data):
        self.ref = data

    """
    This contains the simulation logic which basically just sets the act pos
    to the value of ref pos after some time. Every tick increments (or decrements)
    the joint position value towards the ref pos.
    """

    def joint_state_ticker(self):

        if not self.act.position == self.ref.position:

            """
            Robots move in such a way that all joints start moving at the same time
            and they all stop moving at the same time. In order to do this, we had to
            scale the we increment and decrement the values for each joint.
            """
            ssc = [
                abs(self.ref.position[i] - self.act.position[i])
                for i in range(len(self.ref.position))
            ]

            sync_max = max(ssc)
            sync_max_factor = 1

            if sync_max != 0:
                sync_max_factor = 1 / sync_max

            ssc = [ssc[i] * sync_max_factor for i in range(len(self.ref.position))]

            """
            The act and ref pos comparison is done here. Also the speed scaling. 
            """
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

        """
        Finally, after a message is composed, it is published to the topic for every tick.
        """
        time = Time()
        now = self.get_clock().now().seconds_nanoseconds()
        time.sec = now[0]
        time.nanosec = now[1]
        self.act.header.stamp = time
        self.joint_state_publisher_.publish(self.act)


"""
The main function initializes the simulator class 
and keeps it alive until it is time to terminate.
"""


def main(args=None):
    rclpy.init(args=args)
    robot_simulator = RobotSimulator()

    rclpy.spin(robot_simulator)
    robot_simulator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
