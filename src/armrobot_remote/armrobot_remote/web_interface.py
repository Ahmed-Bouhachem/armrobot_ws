#!/usr/bin/env python3
"""Flask-based web dashboard for armrobot joints and gripper."""
from __future__ import annotations

import threading
from typing import Dict, List

from flask import Flask, redirect, render_template_string, request, url_for

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


HTML_TEMPLATE = """
<!doctype html>
<html>
  <head>
    <title>Armrobot Dashboard</title>
    <style>
      body { font-family: Arial, sans-serif; margin: 2rem; }
      table { border-collapse: collapse; margin-bottom: 1.5rem; }
      th, td { padding: 0.5rem 1rem; border: 1px solid #ddd; text-align: center; }
      form { display: inline; }
      button { padding: 0.4rem 0.8rem; margin: 0.1rem; }
    </style>
  </head>
  <body>
    <h1>Armrobot Web Control</h1>
    <p>Current joint positions (rad):</p>
    <table>
      <tr>
        {% for joint in joints %}
          <th>{{ joint }}<br/>{{ positions.get(joint, 'n/a')|round(3) }}</th>
        {% endfor %}
      </tr>
      <tr>
        {% for joint in joints %}
        <td>
          <form method="post" action="{{ url_for('move_joint') }}">
            <input type="hidden" name="joint" value="{{ joint }}" />
            <input type="hidden" name="delta" value="{{ delta }}" />
            <button type="submit">+{{ delta }}</button>
          </form>
          <form method="post" action="{{ url_for('move_joint') }}">
            <input type="hidden" name="joint" value="{{ joint }}" />
            <input type="hidden" name="delta" value="-{{ delta }}" />
            <button type="submit">-{{ delta }}</button>
          </form>
        </td>
        {% endfor %}
      </tr>
    </table>

    <h2>Gripper</h2>
    <p>Joint 4 position: {{ positions.get('joint_4', 'n/a')|round(3) }}</p>
    <form method="post" action="{{ url_for('set_gripper') }}">
      <input type="hidden" name="position" value="{{ gripper_close }}" />
      <button type="submit">Close</button>
    </form>
    <form method="post" action="{{ url_for('set_gripper') }}">
      <input type="hidden" name="position" value="{{ gripper_open }}" />
      <button type="submit">Open</button>
    </form>
  </body>
</html>
"""


class WebArmController(Node):
    def __init__(self) -> None:
        super().__init__("armrobot_web_interface")
        self.declare_parameter("joint_names", ["joint_1", "joint_2", "joint_3"])
        self.declare_parameter("delta", 0.1)
        self.declare_parameter("arm_action", "/arm_controller/follow_joint_trajectory")
        self.declare_parameter("gripper_action", "/gripper_controller/follow_joint_trajectory")
        self.declare_parameter("gripper_open", 0.0)
        self.declare_parameter("gripper_close", -0.8)
        self.declare_parameter("goal_time", 1.5)
        self.declare_parameter("joint_lower_limits", [-1.57, -1.57, -1.57])
        self.declare_parameter("joint_upper_limits", [1.57, 1.57, 1.57])

        self.joint_names: List[str] = list(
            self.get_parameter("joint_names").get_parameter_value().string_array_value
        )
        self.delta: float = float(self.get_parameter("delta").value)
        self.goal_time: float = float(self.get_parameter("goal_time").value)
        lower_param = self.get_parameter("joint_lower_limits").get_parameter_value().double_array_value
        upper_param = self.get_parameter("joint_upper_limits").get_parameter_value().double_array_value
        self.lower_limits: List[float] = list(lower_param)
        self.upper_limits: List[float] = list(upper_param)
        self.gripper_open = float(self.get_parameter("gripper_open").value)
        self.gripper_close = float(self.get_parameter("gripper_close").value)

        arm_action = self.get_parameter("arm_action").value
        gripper_action = self.get_parameter("gripper_action").value

        self.arm_client = ActionClient(self, FollowJointTrajectory, arm_action)
        self.gripper_client = ActionClient(self, FollowJointTrajectory, gripper_action)

        self.joint_positions: Dict[str, float] = {}
        self.joint_lock = threading.Lock()

        self.create_subscription(JointState, "joint_states", self._joint_state_cb, 10)

    def wait_for_servers(self) -> None:
        self.get_logger().info("Waiting for controller action servers...")
        self.arm_client.wait_for_server()
        self.gripper_client.wait_for_server()
        self.get_logger().info("Controller action servers ready")

    def _joint_state_cb(self, msg: JointState) -> None:
        with self.joint_lock:
            for name, pos in zip(msg.name, msg.position):
                if name in self.joint_names or name == "joint_4":
                    self.joint_positions[name] = pos

    def get_positions(self) -> Dict[str, float]:
        with self.joint_lock:
            return dict(self.joint_positions)

    def jog_joint(self, joint: str, delta: float) -> None:
        if joint not in self.joint_names:
            self.get_logger().warning("Unknown joint %s", joint)
            return
        with self.joint_lock:
            current = self.joint_positions.get(joint, 0.0)
        idx = self.joint_names.index(joint)
        lower = self.lower_limits[idx] if idx < len(self.lower_limits) else None
        upper = self.upper_limits[idx] if idx < len(self.upper_limits) else None
        target = current + delta
        if lower is not None:
            target = max(lower, target)
        if upper is not None:
            target = min(upper, target)

        # Build trajectory matching current positions for other joints
        with self.joint_lock:
            current_positions = [self.joint_positions.get(name, 0.0) for name in self.joint_names]
        current_positions[idx] = target
        traj = JointTrajectory()
        traj.joint_names = list(self.joint_names)
        point = JointTrajectoryPoint()
        point.positions = current_positions
        point.time_from_start = Duration(seconds=self.goal_time).to_msg()
        traj.points.append(point)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        self.arm_client.send_goal_async(goal)

    def set_gripper(self, position: float) -> None:
        traj = JointTrajectory()
        traj.joint_names = ["joint_4"]
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start = Duration(seconds=max(self.goal_time / 2.0, 0.5)).to_msg()
        traj.points.append(point)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        self.gripper_client.send_goal_async(goal)


app = Flask(__name__)
controller: WebArmController | None = None


@app.route("/", methods=["GET"])
def index():
    assert controller is not None
    positions = controller.get_positions()
    return render_template_string(
        HTML_TEMPLATE,
        joints=controller.joint_names,
        positions=positions,
        delta=controller.delta,
        gripper_open=controller.gripper_open,
        gripper_close=controller.gripper_close,
    )


@app.route("/move_joint", methods=["POST"])
def move_joint():
    assert controller is not None
    joint = request.form.get("joint", "")
    delta = float(request.form.get("delta", "0"))
    controller.jog_joint(joint, delta)
    return redirect(url_for("index"))


@app.route("/set_gripper", methods=["POST"])
def set_gripper():
    assert controller is not None
    position = float(request.form.get("position", "0"))
    controller.set_gripper(position)
    return redirect(url_for("index"))


def main(args=None) -> None:
    global controller
    rclpy.init(args=args)
    controller = WebArmController()
    executor = SingleThreadedExecutor()
    executor.add_node(controller)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        controller.wait_for_servers()
        app.run(host="0.0.0.0", port=5000, debug=False)
    finally:
        executor.shutdown()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
