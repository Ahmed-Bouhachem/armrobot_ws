#!/usr/bin/env python3
"""Teach-and-playback node using MoveIt to store and replay named poses."""
from __future__ import annotations

from typing import Dict, List

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

from armrobot_msgs.srv import TeachPose

import moveit_commander
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface


class TeachAndPlayback(Node):
    """Stores named joint configurations and replays them via MoveIt."""

    def __init__(self) -> None:
        super().__init__('teach_and_playback')

        self.declare_parameter('group_name', 'arm')
        self.declare_parameter('joint_names', ['joint_1', 'joint_2', 'joint_3'])

        self.group_name: str = (
            self.get_parameter('group_name').get_parameter_value().string_value
        )
        joint_param = (
            self.get_parameter('joint_names').get_parameter_value().string_array_value
        )
        self.joint_names: List[str] = list(joint_param) if joint_param else []

        moveit_commander.roscpp_initialize([])
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group = MoveGroupCommander(self.group_name)

        if not self.joint_names:
            # Fall back to active joints defined in MoveIt group
            self.joint_names = self.group.get_active_joints()

        self.get_logger().info(
            f"Teaching joints {self.joint_names} in group '{self.group_name}'"
        )

        self.current_joint_state: Dict[str, float] = {}
        self.create_subscription(JointState, '/joint_states', self._joint_state_cb, 10)

        self.taught_poses: Dict[str, List[float]] = {}

        self.teach_srv = self.create_service(
            TeachPose, 'teach_pose', self.teach_pose_cb
        )
        self.goto_srv = self.create_service(
            TeachPose, 'go_to_pose', self.go_to_pose_cb
        )

        self.get_logger().info('TeachAndPlayback node ready!')

    def _joint_state_cb(self, msg: JointState) -> None:
        for name, position in zip(msg.name, msg.position):
            self.current_joint_state[name] = position

    def _capture_joint_positions(self) -> List[float]:
        positions: List[float] = []
        for joint in self.joint_names:
            if joint not in self.current_joint_state:
                raise KeyError(joint)
            positions.append(self.current_joint_state[joint])
        return positions

    def teach_pose_cb(self, request: TeachPose.Request, response: TeachPose.Response) -> TeachPose.Response:
        pose_name = request.name.strip()
        if not pose_name:
            response.success = False
            response.message = 'Pose name must not be empty.'
            return response

        try:
            joint_positions = self._capture_joint_positions()
        except KeyError as exc:
            response.success = False
            response.message = f'Missing joint state for {str(exc)}'
            return response

        self.taught_poses[pose_name] = joint_positions
        self.get_logger().info(f"Stored pose '{pose_name}': {joint_positions}")

        response.success = True
        response.message = f"Pose '{pose_name}' taught."
        return response

    def go_to_pose_cb(self, request: TeachPose.Request, response: TeachPose.Response) -> TeachPose.Response:
        pose_name = request.name.strip()
        if pose_name not in self.taught_poses:
            response.success = False
            response.message = f"Pose '{pose_name}' not found."
            return response

        target_positions = self.taught_poses[pose_name]
        self.get_logger().info(f"Planning to taught pose '{pose_name}'")

        self.group.set_joint_value_target(target_positions)
        plan = self.group.plan()

        plan_valid = False
        plan_msg = plan
        if isinstance(plan, tuple):
            success_flag = plan[0]
            plan_msg = plan[1]
            plan_valid = bool(success_flag)
        else:
            try:
                plan_valid = bool(plan) and len(plan.joint_trajectory.points) > 0
            except AttributeError:
                plan_valid = bool(plan)

        if not plan_valid:
            response.success = False
            response.message = f"Planning to '{pose_name}' failed."
            return response

        self.group.execute(plan_msg, wait=True)
        self.group.stop()
        self.group.clear_pose_targets()

        response.success = True
        response.message = f"Reached pose '{pose_name}'."
        return response

    def destroy_node(self) -> bool:
        moveit_commander.roscpp_shutdown()
        return super().destroy_node()


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
