# Custom Gazebo Models in YOLO-RescueSim

This project uses Gazebo (gz-sim) with a custom world SDF: `project/turtle.sdf`.

When you add models (humans, obstacles, props) to a custom world, there are two parts:

1) Where the model files live (Fuel online vs local folder)
2) How Gazebo discovers them (resource paths)

## Why we install things for models

Model loading is handled by Gazebo and ROS-Gazebo integration packages. You install these so:

- Gazebo can run your world and load SDF models.
- ROS 2 can launch Gazebo and bridge topics (camera, scan, cmd_vel).
- Gazebo can find TurtleBot3 models and any local models you add.

These are not YOLO dependencies; they are simulation / model-loading dependencies.

## Model sources: Fuel vs local

### Option A: Use Fuel (online)
In a world file you can reference Fuel URLs:

- Example: `https://fuel.gazebosim.org/.../models/Ground Plane`

Pros:
- Quick, no local files.

Cons:
- Depends on network / Fuel availability.
- Some models do not behave well with world-level `<include><scale>...`.

### Option B: Use local models (recommended for reliable scaling)
Put model folders under:

- `project/models/<model_name>/`

A local Gazebo model folder typically includes:

- `model.config`
- `model.sdf`
- `meshes/` (DAE / OBJ / STL)
- `materials/` (optional)

Then reference it in the world as:

- `<uri>model://<model_name></uri>`

Example used in this repo:

- `model://walking_person_small`

## How Gazebo finds local models

Gazebo resolves `model://...` using `GZ_SIM_RESOURCE_PATH`.

This repo’s sim launch updates that env var so Gazebo searches:

- The workspace source models folder: `project/models`
- The installed package models folder: `install/project/share/project/models`
- TurtleBot3 Gazebo model folder (for `model://turtlebot3_world`)

If you launch Gazebo outside ROS launch, you must set `GZ_SIM_RESOURCE_PATH` yourself.

## Workflow: copy a Fuel model into the repo

1) Launch once with Fuel so Gazebo downloads the model into Fuel cache.
2) Locate the cached model (example path):
   - `~/.gz/fuel/fuel.gazebosim.org/openrobotics/models/.../model.sdf`
3) Copy the whole model directory into:
   - `project/models/<your_model_name>/`
4) Edit `model.sdf`:
   - Change mesh `<uri>` entries to local paths, for example:
     - `model://<your_model_name>/meshes/<file>`
   - Add mesh scaling where needed:
     - `<scale>0.12 0.12 0.12</scale>`
5) Reference it in `project/turtle.sdf`:
   - `<uri>model://<your_model_name></uri>`
6) Fully restart Gazebo to see changes.

## Notes for humans in this project

- Humans are included in `project/turtle.sdf` as static models.
- The local model `walking_person_small` is scaled in its own `model.sdf` for reliable size changes.
- Collisions were removed from the small human model so TurtleBot does not get blocked by them.

## Troubleshooting

- If you do not see changes: close Gazebo completely and relaunch.
- If Gazebo says it cannot find `model://...`:
  - Confirm the model folder exists under `project/models/`.
  - Confirm the launch file sets `GZ_SIM_RESOURCE_PATH`.
  - Confirm the model folder contains `model.config` and `model.sdf`.
