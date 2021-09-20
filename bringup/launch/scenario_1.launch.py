import os
import json
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


class Panic(Exception):
    def __init__(self, message):
        self.message = message
        super().__init__(self.message)


def generate_launch_description():
    scenario = "scenario_1"
    bringup_dir = FindPackageShare("bringup").find("bringup")
    scenarios_dir = FindPackageShare("scenarios").find("scenarios")
    rviz_config_file = os.path.join(bringup_dir, "config", "scenario_1.rviz")

    tars_parameters_path = os.path.join(
        scenarios_dir, scenario, "robots", "tars", "general.json"
    )

    with open(tars_parameters_path) as jsonfile:
        tars_parameters = json.load(jsonfile)

    case_parameters_path = os.path.join(
        scenarios_dir, scenario, "robots", "case", "general.json"
    )

    with open(case_parameters_path) as jsonfile:
        case_parameters = json.load(jsonfile)

    scenario_parameters = {
        "scenario_parameters_path": os.path.join(
            scenarios_dir,
            scenario,
        ),
        "scenario": scenario,
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
                default_value="",  # parameters["name"],
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
                FindPackageShare("common"),
                "parameters",
                "robot",
                parameters["name"],
                "joint_limits.yaml",
            ]
        )

        kinematics_params = PathJoinSubstitution(
            [
                FindPackageShare("common"),
                "parameters",
                "robot",
                parameters["name"],
                "default_kinematics.yaml",
            ]
        )

        physical_params = PathJoinSubstitution(
            [
                FindPackageShare("common"),
                "parameters",
                "robot",
                parameters["name"],
                "physical_parameters.yaml",
            ]
        )

        visual_params = PathJoinSubstitution(
            [
                FindPackageShare("common"),
                "parameters",
                "robot",
                parameters["name"],
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

    tars_robot_setup = make_robot_description(tars_parameters)
    tars_robot_description = tars_robot_setup[0]
    tars_robot_declared_parameters = tars_robot_setup[1]

    case_robot_setup = make_robot_description(case_parameters)
    case_robot_description = case_robot_setup[0]
    case_robot_declared_parameters = case_robot_setup[1]

    tars_driver_parameters = {
        "ur_address": tars_parameters["ip_address"],
        "tf_prefix": tars_parameters["prefix"],
    }

    case_driver_parameters = {
        "ur_address": tars_parameters["ip_address"],
        "tf_prefix": tars_parameters["prefix"],
    }

    tars_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="ia_planning",
        output="screen",
        parameters=[tars_robot_description],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    case_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="ia_planning",
        output="screen",
        parameters=[case_robot_description],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    tars_simulator_node = Node(
        package="ur_simulator",
        executable="ur_simulator",
        namespace="ia_planning",
        output="screen",
        parameters=[tars_driver_parameters],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    case_simulator_node = Node(
        package="ur_simulator",
        executable="ur_simulator",
        namespace="ia_planning",
        output="screen",
        parameters=[case_driver_parameters],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        namespace="ia_planning",
        output="screen",
        arguments=["-d", rviz_config_file],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    sms_node = Node(
        package="sms",
        executable="sms",
        namespace="ia_planning",
        output="screen",
        parameters=[scenario_parameters],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    nodes_to_start = [
        tars_state_publisher_node,
        case_state_publisher_node,
        tars_simulator_node,
        case_simulator_node,
        sms_node,
        rviz_node,
    ]

    return LaunchDescription(
        tars_robot_declared_parameters + case_robot_declared_parameters + nodes_to_start
    )
