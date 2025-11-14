# armrobot_ws Quickstart

Use the following sequence every time you want to bring the simulation up and move the robot purely through ROS 2 controller commands (MoveIt is optional).

## Build & Source

```bash
cd ~/armrobot_ws
colcon build --packages-select armrobot_description armrobot_controller armrobot_moveit
source install/setup.bash
```

## Launch Gazebo + Controllers

```bash
ros2 launch armrobot_description gazebo.launch.py
```

Wait a few seconds for the robot to spawn and the three controllers to activate (`joint_state_broadcaster`, `arm_controller`, `gripper_controller`). You can confirm in a second terminal:

```bash
source ~/armrobot_ws/install/setup.bash
ros2 control list_controllers
```

All three controllers should show `active`.

## Send Motion Commands (no MoveIt)

With the controllers running, command the joints directly via their FollowJointTrajectory interfaces.

### Arm (joints 1–3)

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

### Gripper (joint 4, joint 5 mimics)

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

Each command should print `Goal successfully reached!` in the terminal and you should see the robot move in Gazebo. Repeat with different target positions/timings as needed.
