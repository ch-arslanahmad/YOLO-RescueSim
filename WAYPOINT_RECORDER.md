# Waypoint Recorder Guide (Teleop + Manual Pose Entry)

Note: Any YOLO/detection-related pieces in this repository are not yet tried or validated end-to-end. This guide focuses only on waypoint recording and playback.

This project currently records waypoints by prompting you to type the robot pose you read from the Gazebo UI.
That sounds manual, but it is the most reliable approach right now because the simulation is not providing a usable ROS pose stream (like `/odom` or TF).

## What you can do with this

- Drive the robot using keyboard teleop.
- Record multiple waypoints in sequence: move → record → move → record.
- Export a CSV in the exact format used by the waypoint navigator.
- Use Demo mode to quickly collect several waypoints with guided prompts.

## Start (two terminals)

Terminal 1 (simulation):

```bash
cd /home/arslan/Desktop/github/YOLO-RescueSim
source /opt/ros/jazzy/setup.bash
source install/setup.bash
./launch_sim.sh
```

Terminal 2 (recorder):

```bash
cd /home/arslan/Desktop/github/YOLO-RescueSim
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 -c "from navigation_scripts.navigation.manual_record_waypoints import main; main()"
```

## Commands (type the letter, then press ENTER)

- `t` Teleop drive mode (real-time key control)
- `r` Record waypoint (prompts you for Gazebo X/Y/theta)
- `p` Print recorded waypoints
- `e` Export to CSV (prints full file path)
- `l` Print last exported CSV path again
- `d` Demo (tries a short move pattern, then prompts you to record after each step)
- `h` Help
- `q` Quit

## Teleop keys (inside Teleop mode)

- `W` forward
- `A` turn left
- `D` turn right
- `S` stop
- `SPACE` record waypoint (it will prompt for Gazebo X/Y/theta)
- `X` exit Teleop back to the command prompt

## How to record coordinates (where do X/Y/theta come from?)

When you press `SPACE` (in teleop) or run command `r`, the recorder will ask you for:

- `X` (meters)
- `Y` (meters)
- `theta` (yaw in radians)

You get these values from the Gazebo UI by selecting the robot model and reading its world pose/position.
If your UI shows yaw in degrees, convert to radians with $\theta = \text{deg} \times \pi / 180$.

### Quick steps to read coordinates in Gazebo

1. Click the robot model (`burger`) in the Gazebo scene.
2. Open the right-side **Inspector / Properties** panel (if it’s hidden).
3. In **Component inspector**, click **Pose** to expand it.
4. Read:
	- **Position** → use `x` and `y` (meters)
	- **Rotation / Orientation** → use **yaw** as `theta` (radians)

If you don’t see an Inspector, many Gazebo UIs also show X/Y in the bottom/status bar when an entity is selected.

#### If Gazebo does NOT show yaw (only quaternion)

Some Gazebo views show orientation as a quaternion (`x, y, z, w`) instead of roll/pitch/yaw.
If that’s what you see, compute yaw ($\theta$) as:

$$
	heta = \operatorname{atan2}(2(wz + xy),\ 1 - 2(y^2 + z^2))
$$

Where $(x,y,z,w)$ are the quaternion values from the inspector.

### Why you need to type these values

The waypoint navigator needs real coordinates to drive to later.
Normally the recorder would read them automatically from ROS pose topics (`/odom` or TF), but in this sim setup those pose signals aren’t reliably available/bridged.
So we use Gazebo as the “source of truth” for pose and manually enter it to produce correct waypoint files.

## Why it does not auto-record the “current position”

Automatic recording requires a ROS pose source, typically:

- `/odom` (`nav_msgs/Odometry`), or
- TF transforms (for example `map -> base_link` or `odom -> base_link`)

In this workspace/sim configuration those pose signals are not reliably available/bridged, so any “auto record” attempt either records zeros or incorrect data.
This manual-entry recorder is the reliable workaround until the pose pipeline is fixed.

## CSV output format

The exported file contains:

```csv
Waypoint_ID,X_Position,Y_Position,Theta_Radians
1,0.5234,1.2567,0.4560
2,1.2345,2.3456,0.7890
```

On export (`e`), the script prints the full path to the CSV. Use `l` later to print it again.

## Auto-move using your recorded waypoints

There are two navigation scripts:

1. **Odom-based navigator** (needs `/odom`):
	- Script: `navigation/waypoint_navigator.py`
	- In this sim setup `/odom` is not reliably available, so it may not work.

2. **Open-loop waypoint player** (does NOT need `/odom`) — recommended for now:

```bash
cd /home/arslan/Desktop/github/YOLO-RescueSim
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 -c "from navigation_scripts.navigation.open_loop_waypoint_player import main; main()"
```

It will ask for your exported CSV path (use `l` in the recorder to print it), then it will drive waypoint-to-waypoint using timed `/cmd_vel` commands.

**Important:** open-loop playback assumes the robot starts at waypoint 1 (no localization), so place the robot at waypoint 1 in Gazebo before you press ENTER to start.
