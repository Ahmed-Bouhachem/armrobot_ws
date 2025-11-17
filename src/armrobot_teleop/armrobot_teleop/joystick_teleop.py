from __future__ import annotations

from typing import Dict, List

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import Joy, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class ArmJoystickTeleop(Node):
    """Listens to /joy and /joint_states and sends FollowJointTrajectory goals."""

    def __init__(self) -> None:
        super().__init__("armrobot_joystick_teleop")

        self.declare_parameter("joint_names", ["joint_1", "joint_2", "joint_3"])
        self.declare_parameter("axis_mapping", [0, 1, 3])
        self.declare_parameter("axis_scale", 0.15)
        self.declare_parameter("deadband", 0.1)
        self.declare_parameter("goal_time", 1.5)
        self.declare_parameter("min_delta", 0.01)
        self.declare_parameter("joint_lower_limits", [-1.57, -1.57, -1.57])
        self.declare_parameter("joint_upper_limits", [1.57, 1.57, 1.57])
        self.declare_parameter("arm_action_name", "/arm_controller/follow_joint_trajectory")
        self.declare_parameter("gripper_action_name", "/gripper_controller/follow_joint_trajectory")
        self.declare_parameter("gripper_open_button", 0)
        self.declare_parameter("gripper_close_button", 1)
        self.declare_parameter("gripper_open_position", 0.0)
        self.declare_parameter("gripper_close_position", -0.8)

        self._joint_names: List[str] = self.get_parameter("joint_names").get_parameter_value().string_array_value
        axis_mapping_param = self.get_parameter("axis_mapping").get_parameter_value().integer_array_value
        self._axis_mapping: List[int] = list(axis_mapping_param)
        self._axis_scale: float = self.get_parameter("axis_scale").value
        self._deadband: float = self.get_parameter("deadband").value
        self._goal_time: float = self.get_parameter("goal_time").value
        self._min_delta: float = self.get_parameter("min_delta").value
        lower_param = self.get_parameter("joint_lower_limits").get_parameter_value().double_array_value
        upper_param = self.get_parameter("joint_upper_limits").get_parameter_value().double_array_value
        self._joint_lower_limits: List[float] = list(lower_param)
        self._joint_upper_limits: List[float] = list(upper_param)

        if len(self._axis_mapping) != len(self._joint_names):
            self.get_logger().warn(
                "axis_mapping length (%d) does not match number of joints (%d); mismatched entries will be ignored",
                len(self._axis_mapping),
                len(self._joint_names),
            )

        arm_action_name = self.get_parameter("arm_action_name").value
        gripper_action_name = self.get_parameter("gripper_action_name").value
        self._gripper_open_button = int(self.get_parameter("gripper_open_button").value)
        self._gripper_close_button = int(self.get_parameter("gripper_close_button").value)
        self._gripper_open_position = float(self.get_parameter("gripper_open_position").value)
        self._gripper_close_position = float(self.get_parameter("gripper_close_position").value)

        self._joint_positions: Dict[str, float] = {}
        self._have_joint_state = False
        self._last_buttons: List[int] = []

        self._arm_client = ActionClient(self, FollowJointTrajectory, arm_action_name)
        self._gripper_client = ActionClient(self, FollowJointTrajectory, gripper_action_name)

        # Subscribe to joint states (for current positions) and joystick inputs.
        self._joint_state_sub = self.create_subscription(JointState, "joint_states", self._joint_state_cb, 10)
        self._joy_sub = self.create_subscription(Joy, "joy", self._joy_cb, 10)

        self._arm_goal_future = None
        self._gripper_goal_future = None

        self._wait_for_servers()

    def _wait_for_servers(self) -> None:
        if not self._arm_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("Waiting for arm controller action server...")
        if not self._gripper_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("Waiting for gripper controller action server...")

    def _joint_state_cb(self, msg: JointState) -> None:
        for name, position in zip(msg.name, msg.position):
            if name in self._joint_names or name == "joint_4":
                self._joint_positions[name] = position
        if len(self._joint_positions) >= len(self._joint_names):
            self._have_joint_state = True

    def _joy_cb(self, msg: Joy) -> None:
        if not self._have_joint_state:
            self.get_logger().throttle(5.0, "Waiting for joint_states before teleop commands")
            self._last_buttons = list(msg.buttons)
            return

        axis_values = msg.axes  # analog values [-1, 1]
        updated_positions: List[float] = []
        deltas_applied = False

        for idx, joint in enumerate(self._joint_names):
            current = self._joint_positions.get(joint)
            if current is None:
                updated_positions.append(0.0)
                continue

            axis_index = self._axis_mapping[idx] if idx < len(self._axis_mapping) else -1
            delta = 0.0
            if 0 <= axis_index < len(axis_values):
                value = axis_values[axis_index]
                if abs(value) > self._deadband:
                    delta = value * self._axis_scale
            target = current + delta
            if delta != 0.0:
                deltas_applied = True
            lower = self._joint_lower_limits[idx] if idx < len(self._joint_lower_limits) else None
            upper = self._joint_upper_limits[idx] if idx < len(self._joint_upper_limits) else None
            if lower is not None:
                target = max(lower, target)
            if upper is not None:
                target = min(upper, target)
            updated_positions.append(target)

        # Only send a new trajectory if something changed enough and the server is ready.
        if deltas_applied and self._arm_client.server_is_ready():
            current_values = [self._joint_positions.get(name, 0.0) for name in self._joint_names]
            max_delta = max(abs(a - b) for a, b in zip(updated_positions, current_values))
            if max_delta >= self._min_delta:
                self._send_arm_goal(updated_positions)

        # buttons for gripper
        buttons = msg.buttons
        if not self._last_buttons:
            self._last_buttons = [0] * len(buttons)

        if self._button_pressed(buttons, self._gripper_close_button):
            self._send_gripper_goal(self._gripper_close_position)
        elif self._button_pressed(buttons, self._gripper_open_button):
            self._send_gripper_goal(self._gripper_open_position)

        self._last_buttons = list(buttons)

    def _button_pressed(self, buttons: List[int], index: int) -> bool:
        if index >= len(buttons):
            return False
        previous = self._last_buttons[index] if index < len(self._last_buttons) else 0
        return buttons[index] == 1 and previous == 0

    def _send_arm_goal(self, target_positions: List[float]) -> None:
        traj = JointTrajectory()
        traj.joint_names = list(self._joint_names)
        point = JointTrajectoryPoint()
        point.positions = target_positions
        point.time_from_start = Duration(seconds=self._goal_time).to_msg()
        traj.points.append(point)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        self.get_logger().debug("Sending arm goal: %s", target_positions)
        future = self._arm_client.send_goal_async(goal)
        future.add_done_callback(self._arm_goal_response_cb)
        self._arm_goal_future = future

    def _arm_goal_response_cb(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Arm goal rejected")
            return
        self.get_logger().debug("Arm goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._arm_result_cb)

    def _arm_result_cb(self, future) -> None:
        result = future.result()
        if result.status != 0:
            self.get_logger().warn("Arm goal failed with status %d", result.status)
        else:
            self.get_logger().debug("Arm goal succeeded")

    def _send_gripper_goal(self, position: float) -> None:
        traj = JointTrajectory()
        traj.joint_names = ["joint_4"]
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start = Duration(seconds=max(self._goal_time / 2.0, 0.5)).to_msg()
        traj.points.append(point)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        if not self._gripper_client.server_is_ready():
            self.get_logger().warn_throttle(5.0, "Waiting for gripper controller action server...")
            return

        self.get_logger().debug("Sending gripper goal: %.3f", position)
        future = self._gripper_client.send_goal_async(goal)
        future.add_done_callback(self._gripper_goal_response_cb)
        self._gripper_goal_future = future

    def _gripper_goal_response_cb(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Gripper goal rejected")
            return
        self.get_logger().debug("Gripper goal accepted")
        goal_handle.get_result_async()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArmJoystickTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
