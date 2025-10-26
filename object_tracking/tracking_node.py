#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge

class TrackerNode(Node):
    def __init__(self):
        super().__init__("tracker_node")
        path = get_package_share_directory("ultralytics_ros")
        yolo_model = "yolov8n.pt"
        self.model = YOLO(f"{path}/models/{yolo_model}")
        self.model.fuse()

        self.bridge = CvBridge()
        self.create_subscription(Image,"/kitti/camera/color/left/image_raw",self.callback,1)
        self.result_pub = self.create_publisher(Image,"tracking_result",1)
    
    def callback(self,msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        classes = list(range(10))
        conf_thres = 0.25
        iou = 0.5
        max_det =100
        tracker = "bytetrack.yaml"
        device = "cpu"
        results = self.model.track(
            source=img,
            conf=conf_thres,
            iou=iou,
            max_det=max_det,
            classes=classes,
            tracker=tracker,
            device=device,
            verbose=False,
            retina_masks=True,
        )
        if results is not None:
           plotted_image = results[0].plot(
                conf=True, line_width=1, font_size=1, labels=True, boxes=True)
           result_image_msg = self.bridge.cv2_to_imgmsg(plotted_image,encoding="bgr8")
           result_image_msg.header=msg.header
           self.result_pub.publish(result_image_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()