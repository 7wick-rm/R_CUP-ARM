import os
import rclpy
from rclpy.node import Node
from ultralytics import YOLO
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray
from ament_index_python.packages import get_package_share_directory


class YOLOInferenceNode(Node):
    def __init__(self):
        super().__init__("yolo_inference_node")

        package_share_dir = get_package_share_directory("gazebo_perception")

        default_model_path = os.path.join(package_share_dir, "weights", "best.pt")

        self.declare_parameter("model_path", default_model_path)

        self.bridge = CvBridge()
        self.logger = self.get_logger()

        self.declare_parameter("camera_input_topic", "/rgb_camera/image_raw")
        self.declare_parameter("annotated_image_topic", "/vision/annotated_image")
        self.declare_parameter("detection_topic", "/vision/detected_objects")

        self.camera_input_topic = (
            self.get_parameter("camera_input_topic").get_parameter_value().string_value
        )
        self.annotated_image_topic = (
            self.get_parameter("annotated_image_topic").get_parameter_value().string_value
        )
        self.detection_topic = (
            self.get_parameter("detection_topic").get_parameter_value().string_value
        )
        self.model_path = self.get_parameter("model_path").get_parameter_value().string_value

        self.logger.info("Loading Model")
        self.model = YOLO(model=self.model_path)
        self.logger.info("Model loaded successfully")

        self.subscriber_ = self.create_subscription(
            Image, self.camera_input_topic, self.image_callback, 10
        )
        self.annotated_pub = self.create_publisher(Image, self.annotated_image_topic, 10)
        self.detection_pub = self.create_publisher(Detection2DArray, self.detection_topic, 10)
        self.get_logger().info("MUJOCO_WS VERSION")

    def image_callback(self, msg):

        cv_image = self.bridge.imgmsg_to_cv2(img_msg=msg, desired_encoding="bgr8")

        results = self.model.track(cv_image, verbose=False, persist=True, conf=0.7)

        annotated_image = results[0].plot()
        annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding="bgr8")

        annotated_msg.header = msg.header

        self.annotated_pub.publish(annotated_msg)

        detection_array = Detection2DArray()
        detection_array.header = msg.header

        if results[0].obb is not None:
            for box in results[0].obb:
                xywhr = box.xywhr[0].cpu().numpy()
                det = Detection2D()
                det.header = msg.header

                det.bbox.center.position.x = float(xywhr[0])
                det.bbox.center.position.y = float(xywhr[1])

                det.bbox.center.theta = float(xywhr[4])

                det.bbox.size_x = float(xywhr[2])
                det.bbox.size_y = float(xywhr[3])

                box_class_id = int(box.cls[0].item())
                box_class = self.model.names[box_class_id]
                if box.id is not None:
                    box_id = str(int(box.id[0].item()))
                else:
                    box_id = str(-1)
                det.id = ".".join([box_class, box_id])

                # self.get_logger().info(det.id)
                detection_array.detections.append(det)

        self.detection_pub.publish(detection_array)


def main(args=None):
    rclpy.init(args=args)
    node = YOLOInferenceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
