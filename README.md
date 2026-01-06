# YOLO-RescueSim

## Idea

### Survivor Detection Using YOLO-Lite on Edge Devices

Robots run lightweight YOLO for detecting humans/rescue markers using ROS topics and Gazebo camera plugin.

## Project Requirenments

- Lightweight YOLO model (YOLO-Lite)
Basically a robot (with camera) that detects humans in a field from a picture.

Or more extensively, rescue markers (like colored flags).

A mobile robot equipped with a camera that identifies humans and rescue markers (such as colored flags) in a given environment by analyzing captured images, using a lightweight YOLO detection model within ROS and Gazebo

Project milestones and changes over time are tracked in [docs/implementation.md](docs/implementation.md).

## Docs

- Setup: [docs/setup.md](docs/setup.md)
- Launch guide: [docs/launch.md](docs/launch.md)
- YOLO setup: [docs/yolo.md](docs/yolo.md)
- Commands reference: [docs/commands.md](docs/commands.md)
- Navigation: [docs/navigation.md](docs/navigation.md)
- Waypoints: [docs/waypoints.md](docs/waypoints.md)
- Custom models: [docs/models.md](docs/models.md)

## Prerequisites

You need:
- Ubuntu 22,24 (LTS or equivilant)
All the required programs only run on it.
- Gazebo (Harmonium recommended)
The actual software for simulation.
- ROS
ROS is simply a term, i may need to install relevent packages like (for YOLO-Lite).


## Flowchart

The following is the algorithm for the robot operation:


## Glossary

## YOLO
YOLO = You Only Look Once.

It’s an object detection algorithm:
- takes an image and outputs bounding boxes
- labels for objects in one pass.

Very fast → can run in real time.

> [!IMPORTANT]
> Think: “Show me all humans in the camera frame, instantly.”

## Edge Devices

“Edge” means computing at the source instead of sending data to a cloud.
- So, a device that takes data and processes it locally to produce output.
