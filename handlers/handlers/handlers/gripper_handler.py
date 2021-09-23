import rclpy
import random
from rclpy.node import Node
from sms_msgs.srv import ManipulateScene
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Transform
from handlers_msgs.msg import CubeState
from handlers_msgs.srv import ChangeCubeState


class GripperHandler(Node):
    def __init__(self):
        super().__init__("gripper_handler")

        self.state_msg = Bool()

        self.name = self.declare_parameter("name", value="robot").value

        self.sms_client = self.create_client(ManipulateScene, "/ia_planning/manipulate_scene")
        self.sms_request = ManipulateScene.Request()
        self.sms_response = ManipulateScene.Response()
        self.sms_call_success = False
        self.sms_call_done = False

        self.cps_client = self.create_client(ChangeCubeState, "/ia_planning/change_cube_state")
        self.cps_request = ManipulateScene.Request()
        self.cps_response = ManipulateScene.Response()
        self.cps_call_success = False
        self.cps_call_done = False

        while not self.sms_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("sms service not available, waiting again...")

        while not self.cps_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("cps service not available, waiting again...")

        self.parent_id = ""
        self.child_id = ""

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
        if (self.act_pos != "home" or self.act_pos != "unknown"):
            self.cube_order[self.act_pos]

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
        if (cube != "empty" or cube != "unknown" or cube != None):
            if data.data:
                self.attach(cube)
            else:
                self.detach(cube)

    def attach(self, cube):
        if not self.gripping:
            self.send_change_parent_request(
                cube, "{robot}_svt_tcp".format(robot=self.name)
            )
            self.gripping = True

    def detach(self, cube):
        if self.gripping:
            self.send_change_parent_request(cube, "world")
            self.gripping = False

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


def main(args=None):
    rclpy.init(args=args)

    client = GripperHandler()
    rclpy.spin(client)

    client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
