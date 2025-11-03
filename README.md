

# 🧠 Object Tracking using YOLOv8 & ROS2

A ROS2 Python package that performs **real-time object detection and tracking** using **Ultralytics YOLOv8**.
The node subscribes to an input image topic, processes frames, and publishes results with **bounding boxes and class labels**.

---

## 🚀 Features

* Uses **YOLOv8** for fast and accurate detection.
* Subscribes to `/camera/image_raw` and publishes `/tracking_result`.
* Works with live camera feeds or **ROS bag files** (e.g., KITTI dataset).

---

## ⚙️ Setup

```bash
pip install ultralytics opencv-python cv_bridge
cd ~/ros2_ws
colcon build --packages-select object_tracking
source install/setup.bash
ros2 run object_tracking tracking_node
```
