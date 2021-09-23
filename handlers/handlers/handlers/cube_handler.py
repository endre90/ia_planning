import rclpy
import random
import time
from rclpy.node import Node
from sms_msgs.srv import ManipulateScene
from geometry_msgs.msg import Transform
from handlers_msgs.msg import CubeState
from handlers_msgs.srv import ChangeCubeState


class CubeHandler(Node):
    def __init__(self):
        super().__init__("cube_handler")

        self.pos_state_msg = CubeState()

        self.sms_client = self.create_client(ManipulateScene, "/ia_planning/manipulate_scene")
        self.sms_request = ManipulateScene.Request()
        self.sms_response = ManipulateScene.Response()
        self.sms_call_success = False
        self.sms_call_done = False

        self.change_state_service = self.create_service(
            ChangeCubeState, "change_cube_state", self.change_state_callback
        )

        self.parent_id = ""
        self.child_id = ""
        self.cube_order = self.randomize_cubes()

        self.state_publisher_ = self.create_publisher(
            CubeState,
            "cube_state",
            10,
        )

        self.state_timer_period = 0.1
        self.state_publisher_timer = self.create_timer(
            self.state_timer_period, self.state_ticker
        )

    def change_state_callback(self, request, response):
        self.cube_order[request.pos] = request.cube
        self.send_change_parent_request(request.pos, request.cube)
        response.result = True
        return response

    def randomize_cubes(self):
        time.sleep(1)
        poses = ["pos1", "pos2", "pos3"]
        cubes = ["red_cube", "blue_cube", "green_cube"]
        order = {
            pose: cubes.pop(cubes.index(random.choice(cubes))) for pose in poses
        }
        for ord in order:
            self.get_logger().info(
                "sms order: {pos} : {cube}".format(pos=ord, cube=order[ord])
            )
            self.send_change_parent_request(order[ord], ord)
        return order

    def state_ticker(self):
        self.pos_state_msg.pos1 = self.cube_order["pos1"]
        self.pos_state_msg.pos2 = self.cube_order["pos2"]
        self.pos_state_msg.pos3 = self.cube_order["pos3"]
        self.state_publisher_.publish(self.pos_state_msg)

    def send_change_parent_request(self, child, parent):

        transform = Transform()
        transform.translation.x = 0.0
        transform.translation.y = 0.0
        transform.translation.z = 0.0
        transform.rotation.x = 0.0
        transform.rotation.y = 0.0
        transform.rotation.z = 0.0
        transform.rotation.w = 1.0

        self.sms_request.remove = False
        self.sms_request.child_frame = child
        self.sms_request.parent_frame = parent
        self.sms_request.transform = transform
        self.sms_request.same_position_in_world = False
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

    client = CubeHandler()
    rclpy.spin(client)

    client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
