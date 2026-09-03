from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():

    gpu_offload = SetEnvironmentVariable(name="__NV_PRIME_RENDER_OFFLOAD", value="1")
    glx_vendor = SetEnvironmentVariable(name="__GLX_VENDOR_LIBRARY_NAME", value="nvidia")



    inference_node = Node(
        package="gazebo_perception",
        executable="inference_node",
        name="inference_node",
        output="screen",
        parameters=[{
            "camera_input_topic": "/zed/zed_node/rgb/color/rect/image"
        }]
    )


    pcl_node = Node(
        package="pcl_geometry",
        executable="pcl_geometry_node",
        name="pcl_geometry_node",
        output="screen",
        parameters=[{
            "point_cloud_topic": "/zed/zed_node/point_cloud/cloud_registered"
        }]
    )

    rviz_node = Node(
        package="rviz2", 
        executable="rviz2", 
        name="rviz", 
        output="log"
    )


    tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        parameters=[{"use_sim_time": True}],
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            "0.57",
            "--yaw",
            "0.0",
            "--pitch",
            "0.89011",  # 39 degrees in radians
            "--roll",
            "0.0",
            "--frame-id",
            "map",
            "--child-frame-id",
            "zed_camera_link",
        ],
    )

    server_node = Node(
        package="gazebo_perception",
        executable="perception_server",
        name="perception_server",
        output="screen",
    )
    
    visualizer_node = Node(
    package="gazebo_perception",
    executable="visualizer_node",
    name="visualizer_node",
    output="screen"
    )

    return LaunchDescription(
        [
            gpu_offload,
            glx_vendor,
            inference_node,
            pcl_node,
            server_node,
            rviz_node,
            #tf_node,
            visualizer_node
        ]
    )
