# armrobot_ws Quickstart

Use these steps to run the Gazebo simulation and move the robot directly through the controllers (without MoveIt).

## 1. Build and Source

```bash
cd ~/armrobot_ws
colcon build --packages-select armrobot_description armrobot_controller armrobot_moveit
source install/setup.bash
```

## 2. Launch Gazebo + Controllers

```bash
ros2 launch armrobot_description gazebo.launch.py
```

Wait until the robot spawns and the controllers report `active`. In another terminal you can confirm with:

```bash
source ~/armrobot_ws/install/setup.bash
ros2 control list_controllers
```

You should see `joint_state_broadcaster`, `arm_controller`, and `gripper_controller` in the `active` state.

## 3. Move the Robot (Direct Controller Commands)

With the controllers running, send trajectories straight to them.

### Arm joints (joint_1, joint_2, joint_3)

```bash
ros2 action send_goal /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{
    trajectory: {
      joint_names: ['joint_1','joint_2','joint_3'],
      points: [
        { positions: [0.5, 0.3, -0.2], time_from_start: {sec: 3} },
        { positions: [0.0, 0.0, 0.0], time_from_start: {sec: 6} }
      ]
    }
  }"
```

### Gripper (joint_4, joint_5 mimics joint_4)

```bash
ros2 action send_goal /gripper_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{
    trajectory: {
      joint_names: ['joint_4'],
      points: [
        { positions: [-0.8], time_from_start: {sec: 2} },
        { positions: [0.0],  time_from_start: {sec: 4} }
      ]
    }
  }"
```

Each command should print `Goal successfully reached!` and the robot should move in Gazebo. Adjust joint targets or timing as needed for your tests.

## 4. Launch MoveIt (optional)

Once Gazebo and the controllers are running, you can bring up the MoveIt pipeline with:

```bash
ros2 launch armrobot_moveit moveit.launch.py
```

MoveIt will start the `move_group` node and RViz so you can plan trajectories interactively. Be sure the controllers are active before sending plans from MoveIt.

## 5. Joystick Teleoperation

Connect a joystick recognized by the `joy` driver (default `/dev/input/js0`) and run:

```bash
ros2 launch armrobot_teleop joystick_teleop.launch.py joy_dev:=/dev/input/js0
```

Move the assigned axes to jog joints 1–3 and use the configured buttons to open/close the gripper. You can tweak scales, limits, and button mapping via ROS parameters (see `armrobot_teleop/armrobot_teleop/joystick_teleop.py`).
