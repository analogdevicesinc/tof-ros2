from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()

    talker_node = Node(
        package="tof_ros2cpp",
        executable="tof_ros2cpp",
    )

    ld.add_action(talker_node)
    return ld