from launch import LaunchDescription
from launch_ros.actions import Node

"""
A launch file can start several nodes at the same time 
and provide each of them with some parameters. Here,
we test how the parameters are set in the robot simulator
nodes and hoe the simulators work for different robot types.
"""

def generate_launch_description():

    tars_joint_names = [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ]

    case_joint_names = ["joint_1", "joint_2", "joint_3"]

    tars_parameters = {
        "name": "tars",
        "joint_names": tars_joint_names,
        "initial_position": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    }

    case_parameters = {
        "name": "case",
        "joint_names": case_joint_names,
        "initial_position": [0.0, 0.0, 0.0],
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
        executable="tars_simulator_dummy",
        namespace="ia_planning",
        output="screen",
        parameters=[tars_parameters],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    case_simulator_node = Node(
        package="robot_simulator",
        executable="robot_simulator",
        namespace="ia_planning",
        output="screen",
        parameters=[case_parameters],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    case_simulator_dummy_node = Node(
        package="dummies",
        executable="case_simulator_dummy",
        namespace="ia_planning",
        output="screen",
        parameters=[case_parameters],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    nodes_to_start = [
        tars_simulator_node,
        case_simulator_node,
        tars_simulator_dummy_node,
        case_simulator_dummy_node,
    ]

    return LaunchDescription(nodes_to_start)
