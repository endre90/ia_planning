import os
import rclpy
import json
from rclpy import executors
import tf2_ros
import time
import threading
import datetime

from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from sms_msgs.srv import ManipulateScene
from sms_msgs.srv import LookupTransform
from geometry_msgs.msg import TransformStamped
from builtin_interfaces.msg import Time
from threading import Lock
# from transformations.transformations import Transformations


class Variables:
    parent = ""
    child = ""
    result = TransformStamped()
    mutex = Lock()
    active_transforms = []
    static_transforms = []


class TFLookupNode(Node):
    def __init__(self):
        super().__init__("tf_lookup")

        self.lookup_tf_rate = 100

        # self.transformations = Transformations()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        self.lookup_tf_rate = 100

        self.create_timer(0.1, self.tf_broadcaster_timer_callback)
        self.create_timer(0.1, self.static_tf_broadcaster_timer_callback)

        self.create_timer(0.05, self.tf_lookup_cb)

        self.get_logger().info("TTFNode started ")

    def tf_lookup_cb(self):
        if Variables.parent != "" and Variables.child != "":
            try:
                self.get_logger().info(
                    "Doing lookup tf: %s to: %s" % (Variables.parent, Variables.child)
                )
                Variables.mutex.acquire()

                Variables.result = self.tf_buffer.lookup_transform(
                    Variables.parent,
                    Variables.child,
                    tf2_ros.Time(seconds=0, nanoseconds=0),
                )
                # Variables.mutex.release()
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ):
                self.get_logger().info(
                    "Failed to lookup tf: %s to: %s"
                    % (Variables.parent, Variables.child)
                )
            else:
                self.get_logger().info(
                    "Success in lookup tf: %s to: %s"
                    % (Variables.parent, Variables.child)
                )
            finally:
                Variables.mutex.release()

    def tf_broadcaster_timer_callback(self):
        # self.mutex.acquire()
        try:
            for tf in Variables.active_transforms:
                if tf != None:
                    if tf != TransformStamped():
                        current_time = self.get_clock().now().seconds_nanoseconds()
                        t = Time()
                        t.sec = current_time[0]
                        t.nanosec = current_time[1]
                        tf.header.stamp = t
                        self.tf_broadcaster.sendTransform(tf)
        finally:
            pass
            # self.mutex.release()

    def static_tf_broadcaster_timer_callback(self):
        # self.mutex.acquire()
        try:
            for tf in Variables.static_transforms:
                self.static_tf_broadcaster.sendTransform(tf)
        finally:
            pass
            # self.mutex.release()

class SceneManipulationService(Node):
    def __init__(self):
        super().__init__("scene_manipulation_service")

        self.mutex = Lock()

        self.sms_service = self.create_service(
            ManipulateScene, "manipulate_scene", self.sms_callback
        )

        self.tf_lookup_service = self.create_service(
            LookupTransform, "lookup_tf", self.tf_lookup_callback
        )

        self.scene_parameters_path = self.declare_parameter(
            "scenario_path", "default value"
        )
        self.scene_parameters_path = (
            self.get_parameter("scenario_path")
            .get_parameter_value()
            .string_value
        )

        self.included_items_paths = []
        for thing in os.listdir(os.path.join(self.scene_parameters_path)):
            thing_parameters = {}
            with open(
                os.path.join(self.scene_parameters_path, thing)
            ) as jsonfile:
                thing_parameters = json.load(jsonfile)
            if thing_parameters["show"]:
                self.included_items_paths.append(
                    os.path.join(self.scene_parameters_path, thing)
                )

        self.included_items = [json.load(open(x)) for x in self.included_items_paths]

        self.active_frames = []
        self.static_frames = []
        self.active_transforms = []
        self.static_transforms = []
        self.active_items = []
        self.static_items = []

        for item in self.included_items:
            tf = TransformStamped()
            tf.header.frame_id = item["parent_frame"]
            tf.header.stamp = Time()
            tf.child_frame_id = item["child_frame"]
            tf.transform = self.make_transform_from_json(item)
            if item["active"]:
                self.active_frames.append(item["child_frame"])
                self.active_transforms.append(tf)
                self.active_items.append(item)
            else:
                self.static_frames.append(item["child_frame"])
                self.static_transforms.append(tf)
                self.static_items.append(item)

        Variables.active_transforms = self.active_transforms
        Variables.static_transforms = self.static_transforms

    def tf_lookup(self, parent, child, timeout):
        deadline = datetime.datetime.now() + datetime.timedelta(milliseconds=timeout)
        Variables.mutex.acquire()
        Variables.result = TransformStamped()
        Variables.parent = parent
        Variables.child = child
        Variables.mutex.release()

        while rclpy.ok():
            time.sleep(0.05)
            if datetime.datetime.now() < deadline:
                if Variables.result != TransformStamped():
                    Variables.mutex.acquire()
                    Variables.parent = ""
                    Variables.child = ""
                    Variables.mutex.release()
                    self.get_logger().info(
                        "Lookuped up tf: %s to: %s" % (parent, child)
                    )
                    return (True, Variables.result)
            else:
                self.get_logger().info(
                    "Failed to lookup tf, deadline expired: %s to: %s" % (parent, child)
                )
                return (False, TransformStamped())

    def tf_lookup_callback(self, request, response):
        (response.success, response.transform) = self.tf_lookup(
            request.parent_id, request.child_id, request.deadline
        )
        self.get_logger().info("responding with: %s" % response)
        return response

    def make_transform(self, transform):
        tf = TransformStamped()
        tf.transform.translation.x = transform.translation.x
        tf.transform.translation.y = transform.translation.y
        tf.transform.translation.z = transform.translation.z
        tf.transform.rotation.x = transform.rotation.x
        tf.transform.rotation.y = transform.rotation.y
        tf.transform.rotation.z = transform.rotation.z
        tf.transform.rotation.w = transform.rotation.w
        return tf.transform

    def make_transform_from_json(self, json):
        tf = TransformStamped()
        tf.transform.translation.x = json["transform"]["translation"]["x"]
        tf.transform.translation.y = json["transform"]["translation"]["y"]
        tf.transform.translation.z = json["transform"]["translation"]["z"]
        tf.transform.rotation.x = json["transform"]["rotation"]["x"]
        tf.transform.rotation.y = json["transform"]["rotation"]["y"]
        tf.transform.rotation.z = json["transform"]["rotation"]["z"]
        tf.transform.rotation.w = json["transform"]["rotation"]["w"]
        return tf.transform

    def sms_callback(self, request, response):
        if (
            request.transform == TransformStamped()
            and not request.same_position_in_world
        ):
            response.success = False
            self.get_logger().info(
                "Not adding empty tf to scene %s to %s"
                % (request.parent_frame, request.child_frame)
            )
            return response
        self.remove = request.remove
        self.request_child_frame = request.child_frame
        self.request_parent_frame = request.parent_frame
        self.request_transform = request.transform
        self.request_same_position_in_world = request.same_position_in_world
        # self.mutex.acquire()
        try:
            self.child_matches = [
                x
                for x in self.active_transforms
                if x != None
                if x.child_frame_id == self.request_child_frame
            ]
            if len(self.child_matches) == 0:
                if not self.remove:
                    tf = TransformStamped()
                    tf.header.frame_id = self.request_parent_frame
                    tf.header.stamp = Time()
                    tf.child_frame_id = self.request_child_frame
                    tf.transform = self.make_transform(self.request_transform)
                    if self.request_child_frame not in self.static_frames:
                        self.active_transforms.append(tf)
                        self.get_logger().info(
                            "Added active child frame '{child}' in parent frame '{parent}'.".format(
                                child=self.request_child_frame,
                                parent=self.request_parent_frame,
                            )
                        )
                    else:
                        self.get_logger().info(
                            "Can't manipulate a static transformation for a child frame '{child}'.".format(
                                child=self.request_child_frame,
                            )
                        )
                        response.result = False
                        return response
                else:
                    self.get_logger().info(
                        "Can't remove a non-existing frame '{child}'.".format(
                            child=self.request_child_frame,
                        )
                    )
            elif len(self.child_matches) == 1:
                tf = TransformStamped()
                tf.header.frame_id = self.request_parent_frame
                tf.header.stamp = Time()
                tf.child_frame_id = self.request_child_frame
                tf.transform = self.make_transform(self.request_transform)
                if self.request_child_frame not in self.static_frames:
                    if not self.request_same_position_in_world:
                        self.active_transforms.remove(self.child_matches[0])
                        if not self.remove:
                            self.active_transforms.append(tf)
                            self.get_logger().info(
                                "Updated active child frame '{child}' in parent frame '{parent}'.".format(
                                    child=self.request_child_frame,
                                    parent=self.request_parent_frame,
                                )
                            )
                        else:
                            self.get_logger().info(
                                "Removed active child frame '{child}' in parent frame '{parent}'.".format(
                                    child=self.request_child_frame,
                                    parent=self.request_parent_frame,
                                )
                            )
                    else:
                        lookup = self.tf_lookup(
                            self.request_parent_frame, self.request_child_frame, 5000
                        )
                        success = lookup[0]
                        new_parent = lookup[1]
                        if not success:
                            self.get_logger().info(
                                "CAN'T CHANGE PARENT FOR '{child}'.".format(
                                    child=self.request_child_frame,
                                )
                            )
                            response.result = False
                            return response
                        self.active_transforms.remove(self.child_matches[0])
                        if not self.remove:
                            self.active_transforms.append(new_parent)
                            self.get_logger().info(
                                "Updated parent for active child frame '{child}' in parent frame '{parent}'.".format(
                                    child=self.request_child_frame,
                                    parent=self.request_parent_frame,
                                )
                            )
                        else:
                            self.get_logger().info(
                                "Removed active child frame '{child}' in parent frame '{parent}'.".format(
                                    child=self.request_child_frame,
                                    parent=self.request_parent_frame,
                                )
                            )
                else:
                    self.get_logger().info(
                        "Can't manipulate a static transformation for a child frame '{child}'.".format(
                            child=self.request_child_frame,
                        )
                    )
                    response.result = False
                    return response
            else:
                self.get_logger().info(
                    "Can't have more than 1 active frame with the name '{child}'.".format(
                        child=self.request_child_frame
                    )
                )
                response.result = False
                return response
        finally:
            Variables.active_transforms = self.active_transforms
            pass
            # self.mutex.release()

        response.result = True
        self.get_logger().info("returning true")
        return response

    


def thread():
    tf = TFLookupNode()
    rclpy.spin(tf)


def main(args=None):
    rclpy.init(args=args)
    try:
        sms = SceneManipulationService()
        tf = TFLookupNode()

        executor = MultiThreadedExecutor()
        executor.add_node(sms)
        executor.add_node(tf)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            sms.destroy_node()
            tf.destroy_node()

    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
