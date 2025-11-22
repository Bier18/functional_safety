from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

def generate_launch_description():
    ld = LaunchDescription()

    fr3_arm_movelt_path = os.path.join(get_package_share_directory('franka_fr3_moveit_config'),
                                       'launch','moveit.launch.py')
    
    run_simulation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fr3_arm_movelt_path),
        launch_arguments={"robot_ip": "dont-care", "use_fake_hardware": "true"}.items()
    )

    state_machine_cmd = Node(
        package="safety_manager",
        executable="state_machine",
        name="state_machine",
        output="screen"
    )

    robot_bridge_cmd = Node(
        package="robot_bridge",
        executable="bridge",
        name="bridge",
        output="screen"
    )

    trajectory_sender_cmd = Node(
        package="safe_controller",
        executable="trajectory_sender",
        name="trajectory_sender",
        output="screen"
    )

    collision_publisher_cmd = Node(
        package="safe_controller",
        executable="collision_publisher",
        name="collision_publisher",
        output="screen"
    )

    safe_controller_cmd = Node(
        package="safe_controller",
        executable="safe_controller",
        name='safe_controller',
        output='screen'
    )

    logger_cmd = Node(
        package="safe_controller",
        executable="logger",
        name='logger',
        output='screen'
    )

    ld.add_action(run_simulation_cmd)
    ld.add_action(state_machine_cmd)
    ld.add_action(robot_bridge_cmd)
    ld.add_action(trajectory_sender_cmd)
    ld.add_action(collision_publisher_cmd)
    ld.add_action(safe_controller_cmd)
    ld.add_action(logger_cmd)
    
    return ld