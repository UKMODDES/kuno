
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


def get_trackbar():
    hue_min = cv2.getTrackbarPos("Hue Min", "TrackedBars")
    hue_max = cv2.getTrackbarPos("Hue Max", "TrackedBars")
    sat_min = cv2.getTrackbarPos("Sat Min", "TrackedBars")
    sat_max = cv2.getTrackbarPos("Sat Max", "TrackedBars")
    val_min = cv2.getTrackbarPos("Val Min", "TrackedBars")
    val_max = cv2.getTrackbarPos("Val Max", "TrackedBars")

    lower = np.array([hue_min, sat_min, val_min])
    upper = np.array([hue_max, sat_max, val_max])
    return lower, upper

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
        print("Identifying ball for hue = " + str(hue))

        image, img = self.get_gripper_image()

        points = []
        while True:
            # Apply mask, with values given by trackbar
            lower, upper = get_trackbar()
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, lower, upper)
            result = cv2.bitwise_and(img, img, mask=mask)

            bw = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
            _, thresh = cv2.threshold(bw, 127, 255, 0)
            blur = cv2.GaussianBlur(thresh, (7,7),0)

            contours, hierarchy = cv2.findContours(blur, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
            #find centre of contours
            points.clear()
            for i in contours:
                M = cv2.moments(i)
                if M['m00'] != 0:
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    cv2.circle(res, (cx, cy), 7, (0, 0, 255), -1)
                    points.append([cx, cy])

            cv2.drawContours(result, contours, -1, (0,255,0), 3)
            cv2.imshow("TrackedBars", result)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        if len(points) == 0:
            return image, None

        point = points[0]
        image_pos = geometry_pb2.Vec2(x=point[0], y=point[1])

        return image, image_pos

    def pick_up_ball(self, image, image_pos):
        # Build the proto
        walk_to = manipulation_api_pb2.WalkToObjectInImage(
            pixel_xy=image_pos,
            transforms_snapshot_for_camera=image.shot.transforms_snapshot,
            frame_name_image_sensor=image.shot.frame_name_image_sensor,
            camera_model=image.source.pinhole,
            offset_distance=None
        )

        # Ask the robot to pick up the object
        walk_to_request = manipulation_api_pb2.ManipulationApiRequest(
            walk_to_object_in_image=walk_to)

        # Send the request
        cmd_response = self.manipulation_api_client.manipulation_api_command(
            manipulation_api_request=walk_to_request)

        # Get feedback from the robot
        while True:
            time.sleep(0.25)
            feedback_request = manipulation_api_pb2.ManipulationApiFeedbackRequest(
                manipulation_cmd_id=cmd_response.manipulation_cmd_id)

            # Send the request
            response = self.manipulation_api_client.manipulation_api_feedback_command(
                manipulation_api_feedback_request=feedback_request)

            print('Current state: ',
                  manipulation_api_pb2.ManipulationFeedbackState.Name(response.current_state))

            if response.current_state == manipulation_api_pb2.MANIP_STATE_DONE:
                break

        print("Finished picking up object")

    def return_to_initial_pos(self):
        # TODO
        pass

    def drop_ball(self):
        # TODO
        pass

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
            time.sleep(1)
            image, image_pos = self.get_ball_image_pos(self.options.hue)
            if image_pos is None:
                break
            self.pick_up_ball(image, image_pos)
            # self.return_to_initial_pos()
            # self.drop_ball()

        print("Powering down")
        self.robot.power_off(cut_immediately=False, timeout_sec=20)

class Options:
    def __init__(self, hue, image_source, scene_pos):
        self.hue = hue
        self.image_source = image_source
        self.scene_pos = scene_pos

def main(argv):
    robot = Robot("192.168.80.3", "Kuno", "olympickuno1")

    # Options for this challenge
    options = Options(
        hue=0,
        image_source="hand_color_image",
        scene_pos=[1.5, 0, 0],)

    robot.run(options)

if __name__ == '__main__':
    if not main(sys.argv[1:]):
        sys.exit(1)
