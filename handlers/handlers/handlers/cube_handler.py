import rclpy
import random
from rclpy.node import Node
from handlers_msgs.msg import CubeState
from handlers_msgs.srv import ChangeCubeState


class CubeHandler(Node):
    def __init__(self):
        super().__init__("cube_handler")

        self.pos_state_msg = CubeState()

        self.change_state_service = self.create_service(
            ChangeCubeState, "change_cube_state", self.change_state_callback
        )

        self.parent_id = ""
        self.child_id = ""

        self.state_publisher_ = self.create_publisher(
            CubeState,
            "cube_state",
            10,
        )

        self.state_timer_period = 0.1
        self.state_publisher_timer = self.create_timer(
            self.state_timer_period, self.state_ticker
        )

        self.cube_order = self.randomize_cubes()

    def change_state_callback(self, request, response):
        self.cube_order[request.pos] = request.cube
        response.result = True
        return response

    def randomize_cubes(self):
        poses = ["pos1", "pos2", "pos3"]
        cubes = ["red_cube", "blue_cube", "green_cube"]
        return {
            pose: cubes.pop(cubes.index(random.choice(cubes))) for pose in poses
        }

    def state_ticker(self):
        self.pos_state_msg.pos1 = self.cube_order["pos1"]
        self.pos_state_msg.pos2 = self.cube_order["pos2"]
        self.pos_state_msg.pos3 = self.cube_order["pos3"]
        self.state_publisher_.publish(self.pos_state_msg)


def main(args=None):
    rclpy.init(args=args)

    client = CubeHandler()
    rclpy.spin(client)

    client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
