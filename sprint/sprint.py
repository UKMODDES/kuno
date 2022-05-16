# Copyright (c) 2022 Boston Dynamics, Inc.  All rights reserved.
#
# Downloading, reproducing, distributing or otherwise using the SDK Software
# is subject to the terms and conditions of the Boston Dynamics Software
# Development Kit License (20191101-BDSDK-SL).

"""Command the robot to go to an offset position using a trajectory command."""

import math
import logging
import sys
import time
from bosdyn.api.basic_command_pb2 import RobotCommandFeedbackStatus
import bosdyn.client
import bosdyn.client.util
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder, blocking_stand, block_for_trajectory_cmd
from bosdyn.api import geometry_pb2 as geo
from bosdyn.api import basic_command_pb2
from bosdyn.client import math_helpers
from bosdyn.client.frame_helpers import ODOM_FRAME_NAME, VISION_FRAME_NAME, BODY_FRAME_NAME, get_se2_a_tform_b

from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.estop import EstopEndpoint, EstopKeepAlive
from bosdyn.api.spot import robot_command_pb2 as robot_command_pb2

_LOGGER = logging.getLogger(__name__)

class Options:
    def __init__(self, hostname, username, password):
        self.hostname = hostname
        self.username = username
        self.password = password

def main():
    options = Options("192.168.80.3", "Kuno", "olympickuno1")
    sdk = bosdyn.client.create_standard_sdk('RobotCommandMaster')
    robot = sdk.create_robot(options.hostname)
    robot.authenticate(options.username, options.password)

    # Create estop
    estop_client = robot.ensure_client("estop")
    estop_endpoint = EstopEndpoint(client=estop_client, name="my_estop", estop_timeout=9.0)
    estop_endpoint.force_simple_setup()
    estop_keep_alive = EstopKeepAlive(estop_endpoint)
    print("HERE")

    # Create lease
    lease_client = robot.ensure_client(LeaseClient.default_service_name)

    # Setup clients for the robot state and robot command services.
    robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
    robot_command_client = robot.ensure_client(RobotCommandClient.default_service_name)

    distance = 2

    with LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
        # Power on the robot and stand it up.
        robot.time_sync.wait_for_sync()
        robot.power_on(timeout_sec=20)
        blocking_stand(robot_command_client)
        execute_sprint(robot_state_client, robot_command_client, distance)
        robot_command_client.robot_command(RobotCommandBuilder.stop_command())


def execute_sprint(robot_state_client, robot_command_client, distance):
    transforms = robot_state_client.get_robot_state().kinematic_state.transforms_snapshot

    waypoints = [
        math_helpers.SE2Pose(x=distance, y=0, angle=0),
        math_helpers.SE2Pose(x=distance, y=0, angle=math.pi),
        math_helpers.SE2Pose(x=0, y=0, angle=math.pi)
    ]

    body_initial = get_se2_a_tform_b(transforms, ODOM_FRAME_NAME, BODY_FRAME_NAME)

    for waypoint in waypoints:
        body_goal = body_initial * waypoint
        robot_cmd = RobotCommandBuilder.synchro_se2_trajectory_point_command(
            goal_x = body_goal.x,
            goal_y = body_goal.y,
            goal_heading = body_goal.angle,
            frame_name = ODOM_FRAME_NAME,
            locomotion_hint = robot_command_pb2.HINT_SPEED_SELECT_TROT)

        end_time = 10.0
        cmd_id = robot_command_client.robot_command(
            lease=None,
            command=robot_cmd,
            end_time_secs=time.time() + end_time)

        # Wait until the robot has reached the goal.
        while True:
            feedback = robot_command_client.robot_command_feedback(cmd_id)
            mobility_feedback = feedback.feedback.synchronized_feedback.mobility_command_feedback
            if mobility_feedback.status != RobotCommandFeedbackStatus.STATUS_PROCESSING:
                print("Failed to reach the goal, stopping")
                return False
            traj_feedback = mobility_feedback.se2_trajectory_feedback
            if (traj_feedback.status == traj_feedback.STATUS_AT_GOAL and
                    traj_feedback.body_movement_status == traj_feedback.BODY_STATUS_SETTLED):
                print("Arrived at waypoint")
                break
    return True

if __name__ == "__main__":
    if not main():
        sys.exit(1)

