import os
import json
from launch import LaunchDescription
from launch import LaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = FindPackageShare("bringup").find("bringup")
    rviz_config_file = os.path.join(bringup_dir, "config", "scenario_1.rviz")

    joint_names = [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ]

    parameters = {
        "name": "tars",
        "ur_type": "ur3e",
        "joint_names": joint_names,
        "initial_position": [0.0, -1.5707, 0.0, 0.0, 0.0, 0.0],
    }

    # home [0.0, -1.5707, 0.0, 0.0, 0.0, 0.0]

    scenario_parameters = {
        "scenario_path": "/home/endre/ia_ws/src/ia_planning/scenarios/scenario_1",
        "scenario": "scenario_1",
    }

    def make_robot_description(parameters):
        declared_arguments = []

        declared_arguments.append(
            DeclareLaunchArgument(
                "ur_type",
                default_value=parameters["ur_type"],
                description="Type/series of used UR robot.",
            )
        )

        declared_arguments.append(
            DeclareLaunchArgument(
                "safety_limits",
                default_value="true",
                description="Enables the safety limits controller if true.",
            )
        )

        declared_arguments.append(
            DeclareLaunchArgument(
                "safety_pos_margin",
                default_value="0.15",
                description="The margin to lower and upper limits in the safety controller.",
            )
        )

        declared_arguments.append(
            DeclareLaunchArgument(
                "safety_k_position",
                default_value="20",
                description="k-position factor in the safety controller.",
            )
        )

        declared_arguments.append(
            DeclareLaunchArgument(
                "description_package",
                default_value="ur_description",
                description="Description package with robot URDF/XACRO files. Usually the argument \
            is not set, it enables use of a custom description.",
            )
        )

        declared_arguments.append(
            DeclareLaunchArgument(
                "description_file",
                default_value="ur.urdf.xacro",
                description="URDF/XACRO description file with the robot.",
            )
        )

        declared_arguments.append(
            DeclareLaunchArgument(
                "prefix",
                default_value=parameters["name"] + "_",
                description="Prefix of the joint names, useful for \
            multi-robot setup. If changed than also joint names in the controllers' configuration \
            have to be updated.",
            )
        )

        ur_type = LaunchConfiguration("ur_type")
        safety_limits = LaunchConfiguration("safety_limits")
        safety_pos_margin = LaunchConfiguration("safety_pos_margin")
        safety_k_position = LaunchConfiguration("safety_k_position")
        description_package = LaunchConfiguration("description_package")
        description_file = LaunchConfiguration("description_file")
        prefix = LaunchConfiguration("prefix")

        joint_limit_params = PathJoinSubstitution(
            [
                FindPackageShare("ur_description"),
                "config",
                parameters["ur_type"],
                "joint_limits.yaml",
            ]
        )

        kinematics_params = PathJoinSubstitution(
            [
                FindPackageShare("ur_description"),
                "config",
                parameters["ur_type"],
                "default_kinematics.yaml",
            ]
        )

        physical_params = PathJoinSubstitution(
            [
                FindPackageShare("ur_description"),
                "config",
                parameters["ur_type"],
                "physical_parameters.yaml",
            ]
        )

        visual_params = PathJoinSubstitution(
            [
                FindPackageShare("ur_description"),
                "config",
                parameters["ur_type"],
                "visual_parameters.yaml",
            ]
        )

        script_filename = PathJoinSubstitution(
            [FindPackageShare(description_package), "resources", "ros_control.urscript"]
        )

        input_recipe_filename = PathJoinSubstitution(
            [
                FindPackageShare(description_package),
                "resources",
                "rtde_input_recipe.txt",
            ]
        )

        output_recipe_filename = PathJoinSubstitution(
            [
                FindPackageShare(description_package),
                "resources",
                "rtde_output_recipe.txt",
            ]
        )

        robot_description_content = Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [FindPackageShare(description_package), "urdf", description_file]
                ),
                " ",
                "joint_limit_params:=",
                joint_limit_params,
                " ",
                "kinematics_params:=",
                kinematics_params,
                " ",
                "physical_params:=",
                physical_params,
                " ",
                "visual_params:=",
                visual_params,
                " ",
                "safety_limits:=",
                safety_limits,
                " ",
                "safety_pos_margin:=",
                safety_pos_margin,
                " ",
                "safety_k_position:=",
                safety_k_position,
                " ",
                "name:=",
                ur_type,
                " ",
                "script_filename:=",
                script_filename,
                " ",
                "input_recipe_filename:=",
                input_recipe_filename,
                " ",
                "output_recipe_filename:=",
                output_recipe_filename,
                " ",
                "prefix:=",
                prefix,
            ]
        )

        robot_description = {"robot_description": robot_description_content}

        return (robot_description, declared_arguments)

    robot_setup = make_robot_description(parameters)
    robot_description = robot_setup[0]
    robot_declared_parameters = robot_setup[1]

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="ia_planning/" + parameters["name"],
        output="screen",
        parameters=[robot_description, {"rate": "20"}],
    )

    robot_simulator_node = Node(
        package="robot_simulator",
        executable="robot_simulator",
        namespace="ia_planning/" + parameters["name"],
        output="screen",
        parameters=[parameters],
    )

    robot_controller_node = Node(
        package="robot_simulator",
        executable="robot_controller",
        namespace="ia_planning/" + parameters["name"],
        output="screen",
        parameters=[parameters],
    )

    robot_simulator_dummy_node = Node(
        package="dummies",
        executable="robot_simulator_dummy",
        namespace="ia_planning/" + parameters["name"],
        output="screen",
        parameters=[parameters],
    )

    nodes_to_start = [
        robot_state_publisher_node,
        robot_simulator_node,
        robot_controller_node,
        # robot_simulator_dummy_node,        
    ]

    return LaunchDescription(robot_declared_parameters + nodes_to_start)
