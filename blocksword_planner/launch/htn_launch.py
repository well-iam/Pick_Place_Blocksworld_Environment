from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([


        # 2) avvia il tuo planner (che importerà dinamicamente lo state)
        Node(
            package='blocksword_planner',
            executable='htn_planner_node',
            output='screen'
        ),
    ])
