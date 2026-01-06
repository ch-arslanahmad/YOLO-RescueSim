# YOLO Environment Setup (Python) for YOLO-RescueSim

This repo runs YOLO in a ROS 2 Python node (`yolo_detector`).

The key constraint is that ROS 2 provides `cv_bridge` as a binary module, and it is sensitive to the NumPy ABI.

## Why we install these (YOLO vs simulation)

There are two separate dependency sets:

1) Simulation dependencies (Gazebo, TurtleBot3, ros_gz bridge)
   - Needed to run the world and publish `/camera/image_raw`.
   - Installed via system packages / ROS installation.

2) YOLO dependencies (Python packages)
   - Needed only for `yolo_detector` inference and image processing.
   - Installed into a Python virtual environment (`.venv`).

## Why we pin NumPy and OpenCV

If your `.venv` installs NumPy 2.x, ROS `cv_bridge` (built against NumPy 1.x on many systems) can crash or throw:

- "A module that was compiled using NumPy 1.x cannot be run in NumPy 2.x"

So we keep:

- `numpy<2`
- `opencv-python<4.12` (OpenCV 4.12+ requires NumPy 2.x)

This is why `requirements-yolo-venv.txt` exists.

## Create and install the YOLO venv

From repo root:

1) Create venv

- `python3 -m venv .venv`
- `source .venv/bin/activate`

2) Install pinned packages

- `pip install -U pip`
- `pip install -r requirements-yolo-venv.txt`

File used:

- `requirements-yolo-venv.txt`

## PyTorch notes

Ultralytics needs PyTorch.

- If you already have Torch installed and YOLO starts, do not change anything.
- If Torch installation tries to pull CUDA / NVIDIA wheels and you do not need GPU, prefer CPU-only Torch.

Exact Torch install commands vary by platform; use the official PyTorch selector for your OS and Python version.

## Quick validation checks

Inside the venv:

- `python -c "import numpy, cv2; import cv_bridge; print('ok')"`

With ROS sourced + workspace overlay + venv:

- `source /opt/ros/jazzy/setup.bash`
- `source install/setup.bash`
- `source .venv/bin/activate`
- `ros2 run project yolo_detector`

## Runtime topics

- Subscribes:
  - `/camera/image_raw` (ROS image bridged from Gazebo)
- Publishes:
  - `/yolo/detections` (JSON in `std_msgs/String`)
  - `/yolo/detection_image` (annotated image)

## View detections in the camera feed

If you want to see bounding boxes in a camera viewer, view the annotated topic:

- Raw camera (no boxes): `/camera/image_raw`
- YOLO annotated camera (boxes): `/yolo/detection_image`

Example (lightweight viewer):

- `ros2 run image_view image_view --ros-args -r image:=/yolo/detection_image`

## Common issues

- YOLO node starts but no detections:
  - Confirm `/camera/image_raw` exists: `ros2 topic list | grep camera`
  - Confirm image bridge is running (from sim launch)

- Crash mentioning NumPy 2.x:
  - In the venv: `pip install -U "numpy<2" "opencv-python<4.12"`
