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
    trigger_request = False
    child = ""
    parent = ""


class ChangeParentRequest(Node):
    def __init__(self):
        super().__init__("change_parent_request")

        self.sms_client = self.create_client(
            ManipulateScene, "/ia_planning/manipulate_scene"
        )
        self.sms_request = ManipulateScene.Request()
        self.sms_response = ManipulateScene.Response()
        self.sms_call_success = False
        self.sms_call_done = False

        while not self.sms_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("sms service not available, waiting again...")

        self.create_timer(0.5, self.call_cps)

        self.get_logger().info("ChangeParentRequest")

    def call_cps(self):
        if Variables.trigger_request and Variables.parent != "" and Variables.child != "":
            self.send_change_parent_request(Variables.child, Variables.parent)
            Variables.trigger_request = False

    def send_change_parent_request(self, child, parent):

        self.sms_request.remove = False
        self.sms_request.child_frame = child
        self.sms_request.parent_frame = parent
        self.sms_request.transform = Transform()
        self.sms_request.same_position_in_world = True
        self.sms_future = self.sms_client.call_async(self.sms_request)
        self.get_logger().info("sms request sent: %s" % self.sms_request)
        while rclpy.ok():
            rclpy.spin_once(self)
            if self.sms_future.done():
                try:
                    response = self.sms_future.result()
                except Exception as e:
                    self.get_logger().info("sms service call failed with: %r" % (e,))
                    self.sms_call_success = False
                else:
                    self.sms_response = response
                    self.get_logger().info(
                        "sms service call succeded with: %s" % self.sms_response
                    )
                    self.sms_call_success = True
                finally:
                    self.get_logger().info(
                        "sms service call done successfully: %s" % self.sms_call_success
                    )
                    self.sms_call_done = True
                break


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
        self.act_pos = "unknown"

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
        cube = self.get_cube()
        self.get_logger().info("CUBE 1: %s" % cube)
        if cube in ["red_cube", "green_cube", "blue_cube"]:
            if data.data:
                self.attach(cube)
            else:
                self.detach(cube)

    def attach(self, cube):
        if not self.gripping:
            Variables.child = cube
            Variables.parent = "{robot}_svt_tcp".format(robot=self.name)
            Variables.trigger_request = True
            self.gripping = True

    def detach(self, cube):
        if self.gripping:
            Variables.child = cube
            Variables.parent = "world"
            Variables.trigger_request = True
            self.gripping = False


def main(args=None):
    rclpy.init(args=args)
    try:
        gh = GripperHandler()
        cpr = ChangeParentRequest()

        executor = MultiThreadedExecutor()
        executor.add_node(gh)
        executor.add_node(cpr)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            gh.destroy_node()
            cpr.destroy_node()

    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
