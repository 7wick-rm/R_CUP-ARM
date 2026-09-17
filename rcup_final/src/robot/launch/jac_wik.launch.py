import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace
from launch.actions import GroupAction
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    robot_pkg_share = get_package_share_directory('robot')
    mtc_pkg_share = get_package_share_directory('mtc_tutorial')

    object_name_arg = DeclareLaunchArgument(
    name="object_name",
    default_value="traffic_light",
    description="Which object piper should assemble"
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_pkg_share, 'launch', 'gazebo.launch.py'))
    )

    pick_place_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mtc_pkg_share, 'launch', 'pick_place_demo.launch.py'))
    )

    piper_pickplace = GroupAction(actions=[
            PushRosNamespace('piper'),
            IncludeLaunchDescription(
                os.path.join(
                    get_package_share_directory('piper_mtc_tutorial'),  # <-- placeholder, see note above
                    'launch', 'pickplace.launch.py'
                ),
                launch_arguments={
                    'object_name': LaunchConfiguration('object_name'),
                }.items()
            ),
        ])

    delayed_pick_place = TimerAction(
        period=35.0,
        actions=[pick_place_launch]
    )

    piper = TimerAction(
        period=37.0,
        actions=[piper_pickplace]
    )

    ld = LaunchDescription()
    ld.add_action(object_name_arg)
    ld.add_action(gazebo_launch)
    ld.add_action(delayed_pick_place)
    ld.add_action(piper)

    return ld
