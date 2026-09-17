import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace
from launch.actions import GroupAction
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    object_name_arg = DeclareLaunchArgument(
        name="object_name",
        default_value="traffic_light",
        description="Which object piper should assemble"
    )

    # --- Warehouse arm's own pick-place stack -- UNCHANGED, launched as-is ---
    # Package/filename assumed based on earlier context in this conversation
    # (mtc_tutorial / pick_place_demo_launch.py) -- confirm these match your
    # actual current package name before trusting this include.
    warehouse_pickplace = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory('mtc_tutorial'),
            'launch', 'pick_place_demo.launch.py'
        )
    )

    # --- Piper's own pick-place stack -- UNCHANGED, launched as-is, just ---
    # --- pushed into the 'piper' namespace to match gazebo.launch.py ------
    #
    # *** BLOCKER, not just a caveat: piper's own launch file spawns a node
    # from package="mtc_tutorial" -- the SAME package name as the warehouse
    # arm's own MTC node. These are two genuinely different packages that
    # happen to share a name because they were built in separate workspaces.
    # You cannot have two ROS packages named "mtc_tutorial" in one combined
    # workspace -- colcon will fail to disambiguate them. Piper's package
    # needs renaming (e.g. "piper_mtc_tutorial") -- update its package.xml,
    # CMakeLists.txt project() call, and this include's package name to
    # match, before this will actually build. I'm using a placeholder name
    # below; replace it with whatever you actually rename it to.
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

    return LaunchDescription([
        object_name_arg,
        warehouse_pickplace,
        piper_pickplace,
    ])