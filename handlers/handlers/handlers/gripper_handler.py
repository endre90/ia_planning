import rclpy
import time
from rclpy.node import Node
from sms_msgs.srv import ManipulateScene
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Transform
from handlers_msgs.msg import CubeState
from handlers_msgs.srv import ChangeCubeState
from rclpy.executors import MultiThreadedExecutor


class Variables:
    trigger_cps_request = False
    trigger_ccs_request = False
    child = ""
    parent = ""
    change_cube_state_pose = ""
    change_cube_state_cube = ""


class ChangeParentRequest(Node):
    def __init__(self):
        super().__init__("change_parent_request")

        self.sms_client = self.create_client(
            ManipulateScene, "/ia_planning/manipulate_scene"
        )

        self.sms_request = ManipulateScene.Request()

        while not self.sms_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("sms service not available, waiting again...")

        self.create_timer(0.5, self.call_cps)

        self.get_logger().info("ChangeParentRequest")

    def call_cps(self):
        if (
            Variables.trigger_cps_request
            and Variables.parent != ""
            and Variables.child != ""
        ):
            self.send_change_parent_request(Variables.child, Variables.parent)
            Variables.trigger_cps_request = False

    def send_change_parent_request(self, child, parent):

        self.sms_request.remove = False
        self.sms_request.child_frame = child
        self.sms_request.parent_frame = parent
        self.sms_request.transform = Transform()
        self.sms_request.same_position_in_world = True
        self.sms_future = self.sms_client.call_async(self.sms_request)
        self.get_logger().info("sms request sent: %s" % self.sms_request)


class ChangeCubeStateRequest(Node):
    def __init__(self):
        super().__init__("change_cube_state_request")

        self.ccs_client = self.create_client(
            ChangeCubeState, "/ia_planning/change_cube_state"
        )

        self.ccs_request = ChangeCubeState.Request()

        while not self.ccs_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("ccs service not available, waiting again...")

        self.create_timer(0.5, self.call_ccs)

        self.get_logger().info("ChangeCubeStateRequest")

    def call_ccs(self):
        if (
            Variables.trigger_ccs_request
            and Variables.change_cube_state_pose != ""
            and Variables.change_cube_state_cube != ""
        ):
            self.send_cube_state_request(
                Variables.change_cube_state_pose, Variables.change_cube_state_cube
            )
            Variables.trigger_ccs_request = False

    def send_cube_state_request(self, pose, cube):

        self.ccs_request.pos = pose
        self.ccs_request.cube = cube
        self.ccs_future = self.ccs_client.call_async(self.ccs_request)
        self.get_logger().info("ccs request sent: %s" % self.ccs_request)


class GripperHandler(Node):
    def __init__(self):
        super().__init__("gripper_handler")

        self.state_msg = Bool()

        self.name = self.declare_parameter("name", value="robot").value

        self.get_logger().info("GripperHandler")

        self.state_publisher_ = self.create_publisher(
            Bool,
            "gripping",
            10,
        )

        self.state_timer_period = 0.1
        self.state_publisher_timer = self.create_timer(
            self.state_timer_period, self.state_ticker
        )

        self.command_subscriber = self.create_subscription(
            Bool,
            "grip",
            self.command_subscriber_callback,
            10,
        )

        self.pos_subscriber = self.create_subscription(
            String,
            "act_pos",
            self.pos_subscriber_callback,
            10,
        )

        self.cube_state_subscriber = self.create_subscription(
            CubeState,
            "/ia_planning/cube_state",
            self.cube_state_subscriber_callback,
            10,
        )

        self.gripping = False
        self.grip = False
        self.pre_grip = False

        self.act_pos = "unknown"
        self.holding_cube = "unknown"

        self.cube_order = {"pos1": "unknown", "pos2": "unknown", "pos3": "unknown"}

    def get_cube(self):
        if any(str(self.act_pos) == x for x in ["pos1", "pos2", "pos3"]):
            return self.cube_order[self.act_pos]

    def state_ticker(self):
        self.state_msg.data = self.gripping
        self.state_publisher_.publish(self.state_msg)

    def pos_subscriber_callback(self, data):
        self.act_pos = data.data

    def cube_state_subscriber_callback(self, data):
        self.cube_order["pos1"] = data.pos1
        self.cube_order["pos2"] = data.pos2
        self.cube_order["pos3"] = data.pos3

    def command_subscriber_callback(self, data):
        self.grip = data.data
        cube = self.get_cube()
        if cube in ["red_cube", "green_cube", "blue_cube"]:
            if self.grip and not self.pre_grip:
                self.attach(cube)
                self.pre_grip = True
        if cube == "empty":
            if self.holding_cube in ["red_cube", "green_cube", "blue_cube"]:
                if not self.grip and self.pre_grip:
                    self.detach(self.holding_cube)
                    self.pre_grip = False

    def attach(self, cube):
        if not self.gripping:
            self.get_logger().info("TRIGGER ATTACH")
            Variables.child = cube
            Variables.parent = "{robot}_svt_tcp".format(robot=self.name)
            Variables.trigger_cps_request = True
            self.gripping = True
            self.holding_cube = cube
            Variables.change_cube_state_pose = self.act_pos
            Variables.change_cube_state_cube = "empty"
            Variables.trigger_ccs_request = True

    def detach(self, cube):
        if self.gripping:
            self.get_logger().info("TRIGGER DETACH")
            Variables.child = cube
            Variables.parent = "world"
            Variables.trigger_cps_request = True
            self.gripping = False
            self.holding_cube = "none"
            Variables.change_cube_state_pose = self.act_pos
            Variables.change_cube_state_cube = cube
            Variables.trigger_ccs_request = True


def main(args=None):
    rclpy.init(args=args)
    try:
        gh = GripperHandler()
        cpr = ChangeParentRequest()
        ccs = ChangeCubeStateRequest()

        executor = MultiThreadedExecutor()
        executor.add_node(gh)
        executor.add_node(cpr)
        executor.add_node(ccs)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            gh.destroy_node()
            cpr.destroy_node()
            ccs.destroy_node()

    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
