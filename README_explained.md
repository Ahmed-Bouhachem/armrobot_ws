# Armrobot Workspace – Code Walkthrough

This document complements the main `README.md` and explains the purpose of the most relevant packages/files that were added recently: the joystick teleoperation package (`armrobot_teleop`) and the Flask-based web interface (`armrobot_remote`). It also summarizes how they interact with the existing controllers.

## Controller background

Both new interfaces send commands through the standard ROS 2 `control_msgs/action/FollowJointTrajectory` interface exposed by the `arm_controller` (joints 1–3) and `gripper_controller` (joint 4). They rely on `/joint_states` to keep track of the current configuration and use the action clients to nudge the robot.

---

## `armrobot_teleop`

Python-based package living in `src/armrobot_teleop`. It uses the `joy` driver to read a physical joystick/gamepad.

### Key files

| File | Purpose |
|------|---------|
| `package.xml` | Declares runtime dependencies: `rclpy`, `rclpy_action`, `control_msgs`, `trajectory_msgs`, `sensor_msgs`, and `joy`. |
| `setup.py` / `setup.cfg` / `resource/armrobot_teleop` | Standard `ament_python` packaging boilerplate so the package installs a console script. |
| `launch/joystick_teleop.launch.py` | Launches `joy_node` plus the teleop node. Arguments: `joy_dev` (defaults to `/dev/input/js0`) and `use_sim_time`. |
| `armrobot_teleop/joystick_teleop.py` | Core node. Subscribes to `/joy` and `/joint_states`, exposes ROS parameters, and publishes action goals. |

### `joystick_teleop.py` in detail

1. **Parameters**
   - `joint_names`: joints controlled (default `joint_1`–`joint_3`).
   - `axis_mapping`: which joystick axis controls each joint.
   - `axis_scale` + `deadband`: how much movement each joystick deflection creates, ignoring small noise.
   - `joint_lower_limits`, `joint_upper_limits`: soft limits enforced in software.
   - `arm_action_name` & `gripper_action_name`: FollowJointTrajectory endpoints.
   - `gripper_*` parameters: open/close button IDs and target angles.

2. **Action clients**
   - Creates two `ActionClient` objects for the arm and gripper controllers and waits for them to become available.

3. **Callbacks**
   - `_joint_state_cb`: updates a thread-safe dictionary of the latest joint angles so teleop knows where the robot currently is.
   - `_joy_cb`: reads axes each time a joystick message arrives, computes new joint targets, clamps to limits, and sends a new trajectory goal if the change exceeds `min_delta`. Also watches button edges to send open/close commands to the gripper.

4. **Goal sending helpers**
   - `_send_arm_goal` / `_send_gripper_goal` build a `JointTrajectory` with a single `JointTrajectoryPoint` and call `send_goal_async`. Completion callbacks log acceptance or failure.

Result: moving the configured axes jogs joints 1–3 incrementally at whatever speed you choose, and the configured buttons operate the gripper.

---

## `armrobot_remote`

Originally provided C++ MoveIt task server components. We added a Flask-based GUI entry point to allow button control via a browser.

### Key additions

| File | Purpose |
|------|---------|
| `armrobot_remote/package.xml` | Now exports `rclpy`, action/message dependencies, and `python3-flask` because the new script uses them. |
| `armrobot_remote/CMakeLists.txt` | Installs the Python GUI scripts (`task_server.py` and `web_interface.py`). |
| `armrobot_remote/armrobot_remote/web_interface.py` | Flask + rclpy hybrid script explained below. |
| `README.md` (section 6) | Documents how to run the GUI (`ros2 run armrobot_remote web_interface.py`). |

### `web_interface.py` in detail

1. **Dual runtime**
   - Starts a rclpy node (`WebArmController`) inside a `SingleThreadedExecutor` on a background thread.
   - Runs a Flask app on the main thread serving HTTP requests.

2. **Node responsibilities**
   - Parameters similar to the joystick node: joint names, nudge delta, trajectory timing, joint limits, gripper open/close angles.
   - Subscribes to `/joint_states` and stores latest positions in a thread-safe dictionary.
   - Maintains two action clients (arm/gripper) and helper methods to send goals.

3. **Flask views**
   - `/`: renders an HTML table with current joint positions plus `+/-` buttons per joint to jog by `delta`. Also provides “Open” and “Close” buttons for the gripper.
   - `/move_joint`: POST endpoint triggered by the `+/-` buttons; calls `controller.jog_joint()`.
   - `/set_gripper`: POST endpoint for open/close commands.

4. **Usage**
   - `ros2 run armrobot_remote web_interface.py`
   - Visit `http://localhost:5000` to interact.

Because the GUI uses the same FollowJointTrajectory actions as the joystick node, it works seamlessly in simulation (and on hardware once controllers are running).

---

## Launch sequence recap

1. `colcon build --packages-select armrobot_description armrobot_controller armrobot_moveit armrobot_remote armrobot_teleop`
2. `source install/setup.bash`
3. Start Gazebo: `ros2 launch armrobot_description gazebo.launch.py`
4. Optional control options:
   - Menu buttons (web): `ros2 run armrobot_remote web_interface.py`
   - Physical joystick: `ros2 launch armrobot_teleop joystick_teleop.launch.py joy_dev:=/dev/input/js0`
   - MoveIt planning: `ros2 launch armrobot_moveit moveit.launch.py`

Use whichever interface suits your testing scenario. All three talk to the same controllers, so they can be launched independently (just avoid commanding simultaneously to prevent conflicting goals).
