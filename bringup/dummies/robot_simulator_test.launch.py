from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    joint_names = [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ]

    tars_parameters = {
        "name": "tars",
        "joint_names": joint_names,
        "initial_position": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    }

    tars_simulator_node = Node(
        package="robot_simulator",
        executable="robot_simulator",
        namespace="ia_planning",
        output="screen",
        parameters=[tars_parameters],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    tars_simulator_dummy_node = Node(
        package="dummies",
        executable="robot_simulator_dummy",
        namespace="ia_planning",
        output="screen",
        parameters=[tars_parameters],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    nodes_to_start = [tars_simulator_node, tars_simulator_dummy_node]

    return LaunchDescription(nodes_to_start)
