import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("chessbot_description"),
                "launch",
                "gazebo_sim.launch.py"
            )
        )
    )
    
    delay_controller = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("chessbot_controller"),
                        "launch",
                        "controller.launch.py"
                    )
                ),
                launch_arguments={"is_sim": "True"}.items()
            )
        ]
    )
    
    delay_moveit = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("chessbot_moveit"),
                        "launch",
                        "moveit.launch.py"
                    )
                ),
                launch_arguments={"is_sim": "True"}.items()
            )
        ]
    )
    
    return LaunchDescription([
        gazebo,
        delay_controller,
        delay_moveit
    ])
