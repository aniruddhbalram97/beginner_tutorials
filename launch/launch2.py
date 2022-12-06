from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
process_rosbag_record = ExecuteProcess(
      cmd=['ros2', 'bag', 'record', '-a'],
)
def generate_launch_description():
    return LaunchDescription([
        process_rosbag_record,
        Node(
            package='cpp_pubsub',
            executable='talker',
            name='talker'
        ),
        Node(
            package='cpp_pubsub',
            executable='listener',
            name='listener'
        ),        
    ])