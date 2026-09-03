#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from perception_msgs.srv import PerceptionSrv
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseArray
from std_msgs.msg import ColorRGBA

class AveragePoseVisualizer(Node):
    def __init__(self):
        super().__init__("average_pose_visualizer")

        # Configuration Parameters
        self.declare_parameter("average_pose_service_topic", "/perception/get_averaged_pose")
        self.declare_parameter("target_samples", 10)
        self.declare_parameter("request_interval_sec", 5.0)

        service_topic = self.get_parameter("average_pose_service_topic").get_parameter_value().string_value
        self.target_samples = self.get_parameter("target_samples").get_parameter_value().integer_value
        request_interval = self.get_parameter("request_interval_sec").get_parameter_value().double_value

        self.client = self.create_client(PerceptionSrv, service_topic)
        
        # Publishers for both MarkerArray (boxes/text) and PoseArray (orientation arrows)
        self.marker_pub = self.create_publisher(MarkerArray, "/perception/averaged_markers", 10)
        self.pose_array_pub = self.create_publisher(PoseArray, "/perception/averaged_pose_array", 10)

        # Timer to periodically request updated averages
        self.timer = self.create_timer(request_interval, self.send_request)
        self.get_logger().info(f"Visualizer Node Initialized. Requesting averages every {request_interval}s.")

    def send_request(self):
        if not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for average pose service...")
            return

        req = PerceptionSrv.Request()
        req.target_samples = self.target_samples
        
        self.get_logger().info(f"Requesting averaged poses ({self.target_samples} samples)...")
        future = self.client.call_async(req)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            self.publish_visualizations(response)
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def publish_visualizations(self, response):
        poses = response.block_poses.poses
        ids = response.block_ids
        classes = response.block_classes
        dims = response.block_dims

        # 1. Publish the PoseArray topic so RViz can render native orientation axes arrows
        pose_array_msg = response.block_poses
        pose_array_msg.header.frame_id = "map"
        pose_array_msg.header.stamp = self.get_clock().now().to_msg()
        self.pose_array_pub.publish(pose_array_msg)

        # 2. Build and publish the MarkerArray (Transparent Cubes + Text Labels)
        marker_array = MarkerArray()

        # Issue a DELETEALL command to clear stale markers from RViz before publishing new ones
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        marker_array.markers.append(delete_all)

        if not poses:
            self.get_logger().info("Received empty response. No blocks to visualize.")
            self.marker_pub.publish(marker_array)
            return

        for i in range(len(ids)):
            cube = Marker()
            cube.header.frame_id = "map"
            cube.header.stamp = pose_array_msg.header.stamp
            cube.ns = "averaged_blocks"
            cube.id = i * 2
            cube.type = Marker.CUBE
            cube.action = Marker.ADD
            cube.pose = poses[i]
            
            cube.scale.x = dims[i].dimensions[0]
            cube.scale.y = dims[i].dimensions[1]
            cube.scale.z = dims[i].dimensions[2]
            
            cube.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.4)

            text = Marker()
            text.header.frame_id = "map"
            text.header.stamp = cube.header.stamp
            text.ns = "averaged_labels"
            text.id = (i * 2) + 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            
            text.pose.position.x = poses[i].position.x
            text.pose.position.y = poses[i].position.y
            text.pose.position.z = poses[i].position.z + (dims[i].dimensions[2] / 2.0) + 0.05
            
            text.text = f"{classes[i]} (ID: {ids[i]})"
            text.scale.z = 0.03 
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.5)

            marker_array.markers.extend([cube, text])

        self.marker_pub.publish(marker_array)
        self.get_logger().info(f"Published {len(ids)} averaged block markers and pose arrays to RViz.")

def main(args=None):
    rclpy.init(args=args)
    node = AveragePoseVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
