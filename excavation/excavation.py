
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

import cv2
import numpy as np
from google.protobuf import wrappers_pb2

import bosdyn.client
import bosdyn.client.estop
import bosdyn.client.lease
import bosdyn.client.util
from bosdyn.api import geometry_pb2, image_pb2, manipulation_api_pb2
from bosdyn.client.image import ImageClient
from bosdyn.client.manipulation_api_client import ManipulationApiClient
from bosdyn.client.robot_command import RobotCommandClient, blocking_stand

g_image_click = None
g_image_display = None

class Options:
    def __init__(self, hostname, username, password):
        self.hostname = hostname
        self.username = username
        self.password = password

def get_image(image_client):
    image_source = "hand_color_image"

    # Take a picture with a camera
    robot.logger.info('Getting an image from: ' + image_source)
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

    return img

def walk_to_object(config):
    robot.logger.info("Powering on robot... This may take a several seconds.")
    robot.power_on(timeout_sec=20)
    assert robot.is_powered_on(), "Robot power on failed."
    robot.logger.info("Robot powered on.")

    # Tell the robot to stand up. The command service is used to issue commands to a robot.
    # The set of valid commands for a robot depends on hardware configuration. See
    # SpotCommandHelper for more detailed examples on command building. The robot
    # command service requires timesync between the robot and the client.
    robot.logger.info("Commanding robot to stand...")
    command_client = robot.ensure_client(RobotCommandClient.default_service_name)
    blocking_stand(command_client, timeout_sec=10)
    robot.logger.info("Robot standing.")

    # TODO: Orient gripper color camera to view scene

    img = get_image(image_client)
    blurred = cv2.GaussianBlur(img, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    image_title = "HSV image"
    cv2.namedWindow(image_title)
    cv2.imshow(image_title, hsv)

    global g_image_click, g_image_display
    g_image_display = hsv 
    cv2.imshow(image_title, g_image_display)
    while g_image_click is None:
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == ord('Q'):
            # Quit
            print('"q" pressed, exiting.')
            exit(0)

    # Show the image to the user and wait for them to click on a pixel
    robot.logger.info('Click on an object to walk up to...')
    image_title = 'Click to walk up to something'
    cv2.namedWindow(image_title)
    cv2.setMouseCallback(image_title, cv_mouse_callback)

    global g_image_click, g_image_display
    g_image_display = img
    cv2.imshow(image_title, g_image_display)
    while g_image_click is None:
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == ord('Q'):
            # Quit
            print('"q" pressed, exiting.')
            exit(0)

    robot.logger.info('Walking to object at image location (' + str(g_image_click[0]) +
                      ', ' + str(g_image_click[1]) + ')')

    walk_vec = geometry_pb2.Vec2(x=g_image_click[0], y=g_image_click[1])

    # Optionally populate the offset distance parameter.
    if config.distance is None:
        offset_distance = None
    else:
        offset_distance = wrappers_pb2.FloatValue(value=config.distance)

    # Build the proto
    walk_to = manipulation_api_pb2.WalkToObjectInImage(
        pixel_xy=walk_vec, transforms_snapshot_for_camera=image.shot.transforms_snapshot,
        frame_name_image_sensor=image.shot.frame_name_image_sensor,
        camera_model=image.source.pinhole, offset_distance=offset_distance)

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

    robot.logger.info('Finished.')
    robot.logger.info('Sitting down and turning off.')

    # Power the robot off. By specifying "cut_immediately=False", a safe power off command
    # is issued to the robot. This will attempt to sit the robot before powering off.
    robot.power_off(cut_immediately=False, timeout_sec=20)
    assert not robot.is_powered_on(), "Robot power off failed."
    robot.logger.info("Robot safely powered off.")


def cv_mouse_callback(event, x, y, flags, param):
global g_image_click, g_image_display
clone = g_image_display.copy()
if event == cv2.EVENT_LBUTTONUP:
    g_image_click = (x, y)
else:
    # Draw some lines on the image.
    #print('mouse', x, y)
    color = (30, 30, 30)
    thickness = 2
    image_title = 'Click to walk up to something'
    height = clone.shape[0]
    width = clone.shape[1]
    cv2.line(clone, (0, y), (width, y), color, thickness)
    cv2.line(clone, (x, 0), (x, height), color, thickness)
    cv2.imshow(image_title, clone)


def arg_float(x):
    try:
        x = float(x)
    except ValueError:
        raise argparse.ArgumentTypeError("%r not a number" % (x))
    return x


def main(argv):
    options = Options("192.168.80.3", "Kuno", "olympickuno1")

    bosdyn.client.util.setup_logging(config.verbose)

    sdk = bosdyn.client.create_standard_sdk('WalkToObjectClient')
    robot = sdk.create_robot(options.hostname)
    robot.authenticate(options.username, options.password)
    robot.time_sync.wait_for_sync()

    assert robot.has_arm(), "Robot requires an arm to run this example."

    # Create estop
    estop_client = robot.ensure_client("estop")
    estop_endpoint = EstopEndpoint(client=estop_client, name="my_estop", estop_timeout=9.0)
    estop_endpoint.force_simple_setup()
    estop_keep_alive = EstopKeepAlive(estop_endpoint)

    lease_client = robot.ensure_client(bosdyn.client.lease.LeaseClient.default_service_name)
    image_client = robot.ensure_client(ImageClient.default_service_name)
    manipulation_api_client = robot.ensure_client(ManipulationApiClient.default_service_name)

    with bosdyn.client.lease.LeaseKeepAlive(image_client, must_acquire=True, return_at_exit=True):
        try:
            execute_excavation(leamanipulation_api_client)
            return True
        except Exception as exc:  # pylint: disable=broad-except
            logger = bosdyn.client.util.get_logger()
            logger.exception("Threw an exception")
            return False


if __name__ == '__main__':
    if not main(sys.argv[1:]):
        sys.exit(1)
