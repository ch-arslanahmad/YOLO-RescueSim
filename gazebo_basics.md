# Gazebo

## Gazebo Navigation

### Via Mouse

| Action              | How to do it                                                        |
| ------------------- | ------------------------------------------------------------------- |
| **Orbit / Rotate**  | **Right mouse button drag** (around point you click)                |
| **Pan / Translate** | **Middle mouse button drag** OR **Shift + Right mouse button drag** |
| **Zoom**            | **Mouse wheel** or **Ctrl + Right mouse button drag up/down**       |
| **Focus on object** | Double-click the object or select it and press **F**                |


### Via Touchpad only

| Action              | Touchpad Gesture / Shortcut                                                            |
| ------------------- | -------------------------------------------------------------------------------------- |
| **Orbit / Rotate**  | **Two-finger drag** (default right-click simulation) OR **Alt + two-finger drag**      |
| **Pan / Translate** | **Shift + two-finger drag** (horizontal + vertical)                                    |
| **Zoom**            | **Pinch in/out** OR **Ctrl + two-finger drag up/down**                                 |
| **Focus on object** | Double-tap on object (some touchpads may require actual click) or select + press **F** |

> [!note]
> Touchpad is more reliable than mouse or keyword (in my experience) for Gazebo navigation.


## Launching Gazebo

You can launch Gazebo by searching in menu:
- Gazebo
- gz
- Gazebo Harmonic

Alternatively, you can launch Gazebo using the following command:

```bash
gz sim
```


To open a file directly in Gazebo, use:

```bash
gz sim <path_to_file>
gz sim my_world.sdf # example
```


## Saving File in Gazebo 

You can save you file in Gazebo by going to:
- File -> Save World As... (make sure the file extension is `.sdf`)

> [!IMPORTANT]
> There are far more commands in gazebo that may be used alongside Plugins, ROS etc, which is covered [here](commands.md).





