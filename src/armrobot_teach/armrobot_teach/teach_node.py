#!/usr/bin/env python3
"""Teach and replay joint configurations using controller action interfaces."""
from __future__ import annotations

from typing import Dict, List

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory

from armrobot_msgs.srv import TeachPose


class TeachAndPlayback(Node):
    """Stores named joint positions and replays them via FollowJointTrajectory."""

    def __init__(self) -> None:
        super().__init__('teach_and_playback')

        self.declare_parameter('joint_names', ['joint_1', 'joint_2', 'joint_3'])
        self.declare_parameter('arm_action', '/arm_controller/follow_joint_trajectory')
        self.declare_parameter('goal_time', 2.0)

        joint_param = (
            self.get_parameter('joint_names').get_parameter_value().string_array_value
        )
        self.joint_names: List[str] = list(joint_param) if joint_param else []
        self.arm_action = self.get_parameter('arm_action').get_parameter_value().string_value
        self.goal_time = float(self.get_parameter('goal_time').value)

        if not self.joint_names:
            raise ValueError('joint_names parameter cannot be empty')

        self.current_joint_state: Dict[str, float] = {}
        self.create_subscription(JointState, '/joint_states', self._joint_state_cb, 10)

        self.arm_client = ActionClient(self, FollowJointTrajectory, self.arm_action)

        self.taught_poses: Dict[str, List[float]] = {}

        self.create_service(TeachPose, 'teach_pose', self._teach_cb)
        self.create_service(TeachPose, 'go_to_pose', self._goto_cb)

        self.get_logger().info(
            f"Teach node listening on {self.arm_action} for joints {self.joint_names}"
        )

    def _joint_state_cb(self, msg: JointState) -> None:
        for name, position in zip(msg.name, msg.position):
            self.current_joint_state[name] = position

    def _capture_current(self) -> List[float]:
        positions: List[float] = []
        for joint in self.joint_names:
            if joint not in self.current_joint_state:
                raise KeyError(joint)
            positions.append(self.current_joint_state[joint])
        return positions

    def _teach_cb(self, request: TeachPose.Request, response: TeachPose.Response) -> TeachPose.Response:
        pose_name = request.name.strip()
        if not pose_name:
            response.success = False
            response.message = 'Pose name must not be empty.'
            return response

        try:
            joint_positions = self._capture_current()
        except KeyError as exc:
            response.success = False
            response.message = f'Missing joint state for {str(exc)}'
            return response

        self.taught_poses[pose_name] = joint_positions
        self.get_logger().info(f"Stored pose '{pose_name}': {joint_positions}")

        response.success = True
        response.message = f"Pose '{pose_name}' taught."
        return response

    def _goto_cb(self, request: TeachPose.Request, response: TeachPose.Response) -> TeachPose.Response:
        pose_name = request.name.strip()
        if pose_name not in self.taught_poses:
            response.success = False
            response.message = f"Pose '{pose_name}' not found."
            return response

        target = self.taught_poses[pose_name]

        if not self.arm_client.wait_for_server(timeout_sec=1.0):
            response.success = False
            response.message = 'Arm controller action server unavailable.'
            return response

        traj = JointTrajectory()
        traj.joint_names = list(self.joint_names)
        point = JointTrajectoryPoint()
        point.positions = list(target)
        point.time_from_start = Duration(seconds=self.goal_time).to_msg()
        traj.points.append(point)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        self.get_logger().info(f"Executing taught pose '{pose_name}'")
        future = self.arm_client.send_goal_async(goal)
        future.add_done_callback(lambda f: self._goal_response_cb(f, pose_name))

        response.success = True
        response.message = f"Sent trajectory for pose '{pose_name}'."
        return response

    def _goal_response_cb(self, future, pose_name: str) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f"Pose '{pose_name}' goal rejected")
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self._goal_result_cb(f, pose_name)
        )

    def _goal_result_cb(self, future, pose_name: str) -> None:
        result = future.result()
        if result.status != 0:
            self.get_logger().warn(
                f"Pose '{pose_name}' execution finished with status {result.status}"
            )
        else:
            self.get_logger().info(f"Reached pose '{pose_name}'")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TeachAndPlayback()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
