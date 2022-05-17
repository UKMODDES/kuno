
# Copyright (c) 2022 Boston Dynamics, Inc.  All rights reserved.
#
# Downloading, reproducing, distributing or otherwise using the SDK Software
# is subject to the terms and conditions of the Boston Dynamics Software
# Development Kit License (20191101-BDSDK-SL).

"""Tutorial to show how to walk the robot to an object, usually in preparation for manipulation.
"""
import argparse
import sys
import time

# Other libraries
import cv2
import numpy as np
from google.protobuf import wrappers_pb2

import bosdyn.client

# Setup
from bosdyn.client.estop import EstopEndpoint, EstopKeepAlive
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive

# Frames
from bosdyn.client import math_helpers
from bosdyn.client.frame_helpers import get_se2_a_tform_b, GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME, VISION_FRAME_NAME, HAND_FRAME_NAME, BODY_FRAME_NAME, get_a_tform_b

# Command and state clients
from bosdyn.client.robot_command import (RobotCommandBuilder, RobotCommandClient,
                                         block_until_arm_arrives, blocking_stand)
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.manipulation_api_client import ManipulationApiClient
from bosdyn.api.basic_command_pb2 import RobotCommandFeedbackStatus

# Image clients
from bosdyn.client.image import ImageClient

# Bosdyn api
from bosdyn.api import geometry_pb2, image_pb2, manipulation_api_pb2
from bosdyn.api import arm_command_pb2, robot_command_pb2, synchronized_command_pb2, trajectory_pb2

from bosdyn.util import seconds_to_duration

def nothing(i):
    pass

g_image_display = None
g_image_click = None

def click_callback(event, x, y, flags, param):
    global g_image_display, g_image_click
    if event == cv2.EVENT_LBUTTONUP:
        g_image_click = (x, y)
    
class Robot:
    def __init__(self, hostname, username, pasword):
        self.sdk = bosdyn.client.create_standard_sdk("Excavation")
        self.robot = self.sdk.create_robot(hostname)
        self.robot.authenticate(username, pasword)
        self.robot.time_sync.wait_for_sync()

        # Start estop
        self.estop_client = self.robot.ensure_client("estop")
        self.estop_endpoint = EstopEndpoint(
            client=self.estop_client,
            name="my-estop",
            estop_timeout=9.0)
        self.estop_endpoint.force_simple_setup()
        self.estop_keep_alive = EstopKeepAlive(self.estop_endpoint)

        # Acquire lease
        self.lease_client = self.robot.ensure_client(LeaseClient.default_service_name)
        self.lease = self.lease_client.acquire()
        self.lease_keep_alive = LeaseKeepAlive(
            self.lease_client,
            must_acquire=True,
            return_at_exit=True)

        # Create clients
        self.robot_state_client = self.robot.ensure_client(
            RobotStateClient.default_service_name)
        self.image_client = self.robot.ensure_client(
            ImageClient.default_service_name)
        self.manipulation_api_client = self.robot.ensure_client(
            ManipulationApiClient.default_service_name)
        self.robot_command_client = self.robot.ensure_client(
            RobotCommandClient.default_service_name)

    def get_initial_flat_body_transform(self):
        robot_state = self.robot_state_client.get_robot_state()
        odom_T_flat_body = get_a_tform_b(
            robot_state.kinematic_state.transforms_snapshot,
            ODOM_FRAME_NAME,
            GRAV_ALIGNED_BODY_FRAME_NAME)
        return odom_T_flat_body

    def look_at_pos(self, frame, pos):
        # Look at a point directly in front
        gaze_target_in_odom = frame.transform_point(
            x=pos[0],
            y=pos[1],
            z=pos[2])

        gaze_command = RobotCommandBuilder.arm_gaze_command(
            gaze_target_in_odom[0],
            gaze_target_in_odom[1],
            gaze_target_in_odom[2],
            ODOM_FRAME_NAME)

        # Make the open gripper RobotCommand
        gripper_command = RobotCommandBuilder.claw_gripper_open_command()

        # Combine the arm and gripper commands into one RobotCommand
        synchro_command = RobotCommandBuilder.build_synchro_command(
            gripper_command, gaze_command)

        # Send the request
        print("Requesting gaze.")
        gaze_command_id = self.command_client.robot_command(synchro_command)
        block_until_arm_arrives(self.command_client, gaze_command_id, 4.0)

    def get_gripper_image(self):
        print('Getting an image from: ' + self.options.image_source)
        image_responses = self.image_client.get_image_from_sources([self.options.image_source])

        if len(image_responses) != 1:
            print('Got invalid number of images: ' + str(len(image_responses)))
            print(image_responses)
            assert False

        image = image_responses[0]
        if image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_DEPTH_U16:
            dtype = np.uint16
        else:
            dtype = np.uint8

        img = np.fromstring(image.shot.image.data, dtype=dtype)
        if image.shot.image.format == image_pb2.Image.FORMAT_RAW:
            img = img.reshape(image.shot.image.rows, image.shot.image.cols)
        else:
            img = cv2.imdecode(img, -1)

        return image, img

    def get_ball_image_pos(self, hue):
        global g_image_click
        print("Identifying ball for hue = " + str(hue))

        image_title = "TrackedBars"
        cv2.namedWindow(image_title)
        cv2.resizeWindow(image_title, 640, 240)
        cv2.setMouseCallback(image_title, click_callback)

        image, img = self.get_gripper_image()

        global g_image_click, g_image_display
        g_image_display = img
        cv2.imshow(image_title, g_image_display)

        g_image_click = None
        while g_image_click is None:
            if cv2.waitKey(1) & 0xFF == ord('q'):
                return None, None

        image_pos = geometry_pb2.Vec2(x=g_image_click[0], y=g_image_click[1])

        return image, image_pos

    def add_grasp_constraint(self, grasp):
        # Specify a vector alignment constraint

        grasp.grasp_params.grasp_params_frame_name = VISION_FRAME_NAME

        # Force the x-axis in the gripper frame to be aligned with -z in the visual frame
        axis_on_gripper_ewrt_gripper = geometry_pb2.Vec3(x=1, y=0, z=0)
        axis_to_align_with_ewrt_vo = geometry_pb2.Vec3(x=0, y=0, z=-1)
        constraint = grasp.grasp_params.allowable_orientation.add()
        constraint.vector_alignment_with_tolerance.axis_on_gripper_ewrt_gripper.CopyFrom(
            axis_on_gripper_ewrt_gripper)
        constraint.vector_alignment_with_tolerance.axis_to_align_with_ewrt_frame.CopyFrom(
            axis_to_align_with_ewrt_vo)

        # We'll take anything within about 10 degrees for top-down or horizontal grasps.
        constraint.vector_alignment_with_tolerance.threshold_radians = 0.17


    def grasp_ball(self, image, image_pos):
        # Build the proto
        grasp = manipulation_api_pb2.PickObjectInImage(
            pixel_xy=image_pos,
            transforms_snapshot_for_camera=image.shot.transforms_snapshot,
            frame_name_image_sensor=image.shot.frame_name_image_sensor,
            camera_model=image.source.pinhole)

        self.add_grasp_constraint(grasp)

        # Ask the robot to pick up the object
        grasp_request = manipulation_api_pb2.ManipulationApiRequest(
            pick_object_in_image=grasp)

        # Send the request
        cmd_response = self.manipulation_api_client.manipulation_api_command(
            manipulation_api_request=grasp_request)

        raycast_attempt = 0
        # Get feedback from the robot
        while True:
            feedback_request = manipulation_api_pb2.ManipulationApiFeedbackRequest(
                manipulation_cmd_id=cmd_response.manipulation_cmd_id)

            # Send the request
            response = self.manipulation_api_client.manipulation_api_feedback_command(
                manipulation_api_feedback_request=feedback_request)

            print('Current state: ',
                  manipulation_api_pb2.ManipulationFeedbackState.Name(response.current_state))

            if response.current_state == manipulation_api_pb2.MANIP_STATE_GRASP_SUCCEEDED:
                break

            if response.current_state == manipulation_api_pb2.MANIP_STATE_GRASP_PLANNING_NO_SOLUTION or response.current_state == manipulation_api_pb2.MANIP_STATE_GRASP_FAILED:
                return False

            if response.current_state == manipulation_api_pb2.MANIP_STATE_ATTEMPTING_RAYCASTING or response.current_state == manipulation_api_pb2.MANIP_STATE_WALKING_TO_OBJECT:
                raycast_attempt += 1
                if raycast_attempt >= 20:
                    print("Stopped after getting stuck on raycasting")
                    return False

            time.sleep(0.25)

        print('Finished grasp.')
        return True

    def move_arm_up(self, frame):
        move_up = 0.3
        move_back = 0.5

        robot_state = self.robot_state_client.get_robot_state()
        initial_pose = get_a_tform_b(
            robot_state.kinematic_state.transforms_snapshot,
            BODY_FRAME_NAME,
            HAND_FRAME_NAME)

        rot = math_helpers.Quat()
        pitch = -0.2
        rot.x = 0
        rot.y = np.sin(pitch/2)
        rot.z = 0
        rot.w = np.cos(pitch/2)

        pose1 = math_helpers.SE3Pose(x=0.55, y=0, z=0.45, rot=rot)

        # Build the trajectory proto by combining the two points
        hand_traj = trajectory_pb2.SE3Trajectory(points=[
            trajectory_pb2.SE3TrajectoryPoint(
                pose=pose1.to_proto(),
                time_since_reference=seconds_to_duration(1.0)
            )
        ])

        # Build the command by taking the trajectory and specifying the frame it is expressed
        # in.
        #
        # In this case, we want to specify the trajectory in the body's frame, so we set the
        # root frame name to the flat body frame.
        arm_cartesian_command = arm_command_pb2.ArmCartesianCommand.Request(
            pose_trajectory_in_task=hand_traj, root_frame_name=GRAV_ALIGNED_BODY_FRAME_NAME)

        # Pack everything up in protos.
        arm_command = arm_command_pb2.ArmCommand.Request(
            arm_cartesian_command=arm_cartesian_command)

        synchronized_command = synchronized_command_pb2.SynchronizedCommand.Request(
            arm_command=arm_command)

        robot_command = robot_command_pb2.RobotCommand(
            synchronized_command=synchronized_command)

        # Keep the gripper closed the whole time.
        robot_command = RobotCommandBuilder.claw_gripper_open_fraction_command(
            0, build_on_command=robot_command)

        print("Sending trajectory command for move arm up")

        # Send the trajectory to the robot.
        cmd_id = self.command_client.robot_command(robot_command)

        # Wait until the arm arrives at the goal.
        while True:
            feedback_resp = self.command_client.robot_command_feedback(cmd_id)
            print(
                'Distance to final point: ' + '{:.2f} meters'.format(
                    feedback_resp.feedback.synchronized_feedback.arm_command_feedback.
                    arm_cartesian_feedback.measured_pos_distance_to_goal) +
                ', {:.2f} radians'.format(
                    feedback_resp.feedback.synchronized_feedback.arm_command_feedback.
                    arm_cartesian_feedback.measured_rot_distance_to_goal))

            if feedback_resp.feedback.synchronized_feedback.arm_command_feedback.arm_cartesian_feedback.status == arm_command_pb2.ArmCartesianCommand.Feedback.STATUS_TRAJECTORY_COMPLETE:
                print('Move complete.')
                break
            time.sleep(0.1)

    def throw_ball(self):
        robot_state = self.robot_state_client.get_robot_state()
        initial_pose = get_a_tform_b(
            robot_state.kinematic_state.transforms_snapshot,
            BODY_FRAME_NAME,
            HAND_FRAME_NAME)

        rot = math_helpers.Quat()
        pitch = -0.1
        rot.x = 0
        rot.y = np.sin(pitch/2)
        rot.z = 0
        rot.w = np.cos(pitch/2)

        move_dist = 0.0

        pose1 = math_helpers.SE3Pose(
            x=initial_pose.x,
            y=initial_pose.y,
            z=initial_pose.z,
            rot=rot)

        pose2 = math_helpers.SE3Pose(
            x=initial_pose.x + move_dist/np.sqrt(2),
            y=initial_pose.y,
            z=initial_pose.z + move_dist/np.sqrt(2),
            rot=rot)

        speed = 1.0
        vel = math_helpers.SE3Velocity(
            lin_x = speed*np.cos(-pitch), lin_y=0, lin_z=speed*np.sin(-pitch), ang_x=0, ang_y=0, ang_z=0)

        # Build the trajectory proto by combining the two points
        hand_traj_point = trajectory_pb2.SE3TrajectoryPoint(
            pose=pose2.to_proto(),
            velocity=vel.to_proto(),
            time_since_reference=seconds_to_duration(1.0)
        )
        hand_traj = trajectory_pb2.SE3Trajectory(points=[hand_traj_point])

        # Build the command by taking the trajectory and specifying the frame it is expressed
        # in.
        #
        # In this case, we want to specify the trajectory in the body's frame, so we set the
        # root frame name to the flat body frame.
        arm_cartesian_command = arm_command_pb2.ArmCartesianCommand.Request(
            pose_trajectory_in_task=hand_traj, root_frame_name=GRAV_ALIGNED_BODY_FRAME_NAME)

        # Pack everything up in protos.
        arm_command = arm_command_pb2.ArmCommand.Request(
            arm_cartesian_command=arm_cartesian_command)

        synchronized_command = synchronized_command_pb2.SynchronizedCommand.Request(
            arm_command=arm_command)

        robot_command = robot_command_pb2.RobotCommand(
            synchronized_command=synchronized_command)

        # Keep the gripper closed the whole time.
        robot_command = RobotCommandBuilder.claw_gripper_open_fraction_command(
            1.0, build_on_command=robot_command)

        print("Sending trajectory command for open")

        # Send the trajectory to the robot.
        cmd_id = self.command_client.robot_command(robot_command)

        # Wait until the arm arrives at the goal.
        time.sleep(1)

    def run_forward(self):
        transforms = self.robot_state_client.get_robot_state().kinematic_state.transforms_snapshot

        print("Running forward")
        distance_forward = 1.0
        waypoint = math_helpers.SE2Pose(x=distance_forward, y=0, angle=0)

        body_initial = get_se2_a_tform_b(transforms, ODOM_FRAME_NAME, BODY_FRAME_NAME)

        body_goal = body_initial * waypoint
        robot_cmd = RobotCommandBuilder.synchro_se2_trajectory_point_command(
            goal_x = body_goal.x,
            goal_y = body_goal.y,
            goal_heading = body_goal.angle,
            frame_name = ODOM_FRAME_NAME)

        end_time = 10.0
        cmd_id = self.robot_command_client.robot_command(
            lease=None,
            command=robot_cmd,
            end_time_secs=time.time() + end_time)

        # Wait until the robot has reached the goal.
        while True:
            feedback = self.robot_command_client.robot_command_feedback(cmd_id)
            mobility_feedback = feedback.feedback.synchronized_feedback.mobility_command_feedback
            if mobility_feedback.status != RobotCommandFeedbackStatus.STATUS_PROCESSING:
                print("Failed to reach the goal, stopping")
                return False
            traj_feedback = mobility_feedback.se2_trajectory_feedback
            if (traj_feedback.status == traj_feedback.STATUS_AT_GOAL and
                    traj_feedback.body_movement_status == traj_feedback.BODY_STATUS_SETTLED):
                print("Arrived at waypoint")
                break

    def run(self, options):
        self.options = options
        print("Powering on")
        self.robot.power_on(timeout_sec=20)
        if not self.robot.is_powered_on():
            print("Failed to power on")
            return False
        print("Robot powered on.")

        print("Standing up")
        self.command_client = self.robot.ensure_client(RobotCommandClient.default_service_name)
        blocking_stand(self.command_client, timeout_sec=10)

        print("Unstowing arm")
        unstow = RobotCommandBuilder.arm_ready_command()
        unstow_command_id = self.command_client.robot_command(unstow)
        print("Unstow command issued, blocking until complete")
        block_until_arm_arrives(self.command_client, unstow_command_id, 3.0)

        initial_flat_body_transform = self.get_initial_flat_body_transform()

        iter_num = 0
        while True:
            print("Starting iteration " + str(iter_num))
            iter_num+=1

            print("Looking at scene")
            self.look_at_pos(initial_flat_body_transform, self.options.scene_pos)
            time.sleep(5)
            image, image_pos = self.get_ball_image_pos(self.options.hue)
            if image is None:
                break
            if image_pos is None:
                print("Failed to find any balls, trying again")
                iter_num-=1
                continue
            success = self.grasp_ball(image, image_pos)
            if not success:
                print("Failed to grasp object, trying again")
                iter_num-=1
                continue
            self.move_arm_up(initial_flat_body_transform)
            self.throw_ball()
            self.run_forward()
            break

        print("Powering down")
        self.robot.power_off(cut_immediately=False, timeout_sec=20)

class Options:
    def __init__(self, hue, image_source, scene_pos):
        self.hue = hue
        self.image_source = image_source
        self.scene_pos = scene_pos

def main():
    robot = Robot("192.168.80.3", "Kuno", "olympickuno1")

    # Options for this challenge
    options = Options(
        hue=0,
        image_source="hand_color_image",
        scene_pos=[1.0, 0, 0],)

    robot.run(options)

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print("{}".format(e))
        sys.exit(1)