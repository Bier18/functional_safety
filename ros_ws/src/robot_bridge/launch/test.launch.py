from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription

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

    ld.add_action(run_simulation_cmd)
    ld.add_action(state_machine_cmd)
    ld.add_action(robot_bridge_cmd)
    
    return ld