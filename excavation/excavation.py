
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
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME, get_a_tform_b

# Command and state clients
from bosdyn.client.robot_command import (RobotCommandBuilder, RobotCommandClient,
                                         block_until_arm_arrives, blocking_stand)
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.manipulation_api_client import ManipulationApiClient

# Image clients
from bosdyn.client.image import ImageClient

# Bosdyn api
from bosdyn.api import geometry_pb2, image_pb2, manipulation_api_pb2


class Robot:
    def __init__(self, hostname, username, pasword):
        self.sdk = bosdyn.client.create_standard_sdk("Excavation")
        self.robot = self.sdk.create_robot(hostname)
        self.robot.authenticate(username, pasword)
        self.robot.time_sync.wait_for_sync()

        # Start estop
        self.estop_client = robot.ensure_client("estop")
        self.estop_endpoint = EstopEndpoint(
            client=self.estop_client,
            name="estop",
            estop_timeout=9.0)

        # Acquire lease
        self.lease_client = robot.ensure_client(LeaseClient.default_service_name)
        self.lease_keep_alive = LeaseKeepAlive(
            self.lease_client,
            must_acquire=True,
            return_at_exit=True)

        # Create clients
        self.robot_state_client = robot.ensure_client(
            RobotStateClient.default_service_name)
        self.image_client = robot.ensure_client(
            ImageClient.default_service_name)
        self.manipulation_api_client = robot.ensure_client(
            ManipulationApiClient.default_service_name)

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
        robot.logger.info("Requesting gaze.")
        gaze_command_id = command_client.robot_command(synchro_command)
        block_until_arm_arrives(command_client, gaze_command_id, 4.0)

    def get_gripper_image(self):
        sef.robot.logger.info('Getting an image from: ' + self.options.image_source)
        image_responses = image_client.get_image_from_sources([image_source])

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

    def show_image(self, img):
        image_title = "Showing image"
        cv2.namedWindow(image_title)
        cv2.imshow(image_title, img)
        while True:
            key = cv2.waitKey(1) & 0xFF
            if key == ord('\n'):
                break

    def get_card_hue(self):
        self.robot.logger.info("Identifying card hue")

        image, img = self.get_gripper_image()
        blurred = cv2.GaussianBlur(img, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        self.show_image(img)

        image_title = "HSV image"
        cv2.namedWindow(image_title)
        cv2.imshow(image_title, hsv)
        while True:
            key = cv2.waitKey(1) & 0xFF
            if key == ord('\n'):
                break

    def get_ball_image_pos(self, hue):
        self.robot.logger.info("Identifying ball for hue = " + str(hue))

        image, img = self.get_gripper_image()
        blurred = cv2.GaussianBlur(img, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        self.show_image(img)

        # TODO
        x = 0
        y = 0
        image_pos = geometry_pb2.Vec2(x, y)

        return image, image_pos

    def pick_up_ball(self, image, image_pos):
        # TODO: Set a different offset_distance?
        offset_distance = None

        # Build the proto
        walk_to = manipulation_api_pb2.WalkToObjectInImage(
            pixel_xy=image_pos,
            transforms_snapshot_for_camera=image.shot.transforms_snapshot,
            frame_name_image_sensor=image.shot.frame_name_image_sensor,
            camera_model=image.source.pinhole,
            offset_distance=offset_distance
        )

        # Ask the robot to pick up the object
        walk_to_request = manipulation_api_pb2.ManipulationApiRequest(
            walk_to_object_in_image=walk_to)

        # Send the request
        cmd_response = manipulation_api_client.manipulation_api_command(
            manipulation_api_request=walk_to_request)

        # Get feedback from the robot
        while True:
            time.sleep(0.25)
            feedback_request = manipulation_api_pb2.ManipulationApiFeedbackRequest(
                manipulation_cmd_id=cmd_response.manipulation_cmd_id)

            # Send the request
            response = manipulation_api_client.manipulation_api_feedback_command(
                manipulation_api_feedback_request=feedback_request)

            print('Current state: ',
                  manipulation_api_pb2.ManipulationFeedbackState.Name(response.current_state))

            if response.current_state == manipulation_api_pb2.MANIP_STATE_DONE:
                break

        self.robot.logger.info("Finished picking up object")

    def return_to_initial_pos(self):
        # TODO
        pass

    def drop_ball(self):
        # TODO
        pass

    def run(self, options):
        self.options = options
        self.robot.logger.info("Powering on")
        self.robot.power_on(timeout_sec=20)
        if not self.robot.is_powered_on():
            self.robot.logger.info("Failed to power on")
            return False
        self.robot.logger.info("Robot powered on.")

        self.robot.logger.info("Standing up")
        self.command_client = robot.ensure_client(RobotCommandClient.default_service_name)
        blocking_stand(self.command_client, timeout_sec=10)

        self.robot.logger.info("Unstowing arm")
        unstow = RobotCommandBuilder.arm_ready_command()
        unstow_command_id = self.command_client.robot_command(unstow)
        self.robot.logger.info("Unstow command issued, blocking until complete")
        block_until_arm_arrives(self.command_client, unstow_command_id, 3.0)

        initial_flat_body_transform = self.get_initial_flat_body_transform()

        for i in range(self.num_iterations):
            self.robot.logger.info("Starting iteration " + str(i))

            self.robot.logger.info("Looking at card")
            self.look_at_pos(initial_flat_body_transform, self.options.card_pos)

            self.robot.logger.info("Starting countdown before taking photo of card", i)
            countdown = 10
            while countdown != 0:
                self.robot.logger.info(str(countdown))
                countdown-=1
                time.sleep(1)

            card_hue = self.get_card_hue()
            self.robot.logger.info("Looking at scene")
            self.look_at_scene(initial_flat_body_transform, self.options.scene_pos)

            image, image_pos = self.get_ball_image_pos(card_hue)
            # self.pick_up_ball(image, image_pos)
            # self.return_to_initial_pos()
            # self.drop_ball()

        self.robot.logger.info("Powering down")
        self.robot.power_off(cut_immediately=False, timeout_sec=20)

class Options:
    def __init__(self, num_colors, image_source, card_pos, scene_pos):
        self.num_iterations = num_iterations
        self.image_source = image_source
        self.card_pos = card_pos
        self.scene_pos = scene_pos

def main(argv):
    robot = Robot("192.168.80.3", "Kuno", "olympickuno1")

    # Options for this challenge
    options = Options(
        num_colors=1,
        image_source="hand_color_image",
        card_pos=[0.5, 0, 0.5],
        scene_pos=[3.0, 0, 0],)

    robot.run(options)

if __name__ == '__main__':
    if not main(sys.argv[1:]):
        sys.exit(1)
