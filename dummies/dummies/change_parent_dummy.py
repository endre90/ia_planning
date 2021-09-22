import rclpy
from rclpy.node import Node
from sms_msgs.srv import ManipulateScene
from geometry_msgs.msg import Transform


class ChangeParentDummy(Node):
    def __init__(self):
        super().__init__("change_parent_dummy")

        # self.transform = Transform()

        self.sms_client = self.create_client(ManipulateScene, "/ia_planning/manipulate_scene")
        self.sms_request = ManipulateScene.Request()
        self.sms_response = ManipulateScene.Response()
        self.sms_call_success = False
        self.sms_call_done = False

        while not self.sms_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("sms service not available, waiting again...")

        self.parent_id = ""
        self.child_id = ""

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

    client = ChangeParentDummy()
    client.send_change_parent_request("green_cube", "tars_svt_tcp")

    client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
