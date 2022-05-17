import cv2
import numpy as np
from google.protobuf import wrappers_pb2

import bosdyn.client

# Setup
from bosdyn.client.estop import EstopEndpoint, EstopKeepAlive
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive

# Frames
from bosdyn.client import math_helpers
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME, VISION_FRAME_NAME, HAND_FRAME_NAME, get_a_tform_b

# Command and state clients
from bosdyn.client.robot_command import (RobotCommandBuilder, RobotCommandClient,
                                         block_until_arm_arrives, blocking_stand)
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.manipulation_api_client import ManipulationApiClient

# Image clients
from bosdyn.client.image import ImageClient

# Bosdyn api
from bosdyn.api import geometry_pb2, image_pb2, manipulation_api_pb2
from bosdyn.api import arm_command_pb2, robot_command_pb2, synchronized_command_pb2, trajectory_pb2


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
from shape_detection import detect_shapes
from colour_detection import filter_all_colours, resize

# Setup
from bosdyn.client.estop import EstopEndpoint, EstopKeepAlive
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive

# Frames
from bosdyn.client import math_helpers
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME, VISION_FRAME_NAME, HAND_FRAME_NAME, BODY_FRAME_NAME, get_a_tform_b

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

def get_trackbar():
    hue_centre = cv2.getTrackbarPos("Hue Centre", "TrackedBars")
    hue_margin = cv2.getTrackbarPos("Hue Margin", "TrackedBars")
    sat_min = cv2.getTrackbarPos("Sat Min", "TrackedBars")
    sat_max = cv2.getTrackbarPos("Sat Max", "TrackedBars")
    val_min = cv2.getTrackbarPos("Val Min", "TrackedBars")
    val_max = cv2.getTrackbarPos("Val Max", "TrackedBars")

    hue_min = hue_centre - hue_margin/2
    if hue_min < 0:
        hue_min += 180
    hue_max_default = hue_centre + hue_margin/2
    if hue_max > 180:
        hue_max -= 180

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
        print("Identifying ball for hue = " + str(hue))

        cv2.namedWindow("TrackedBars")
        cv2.resizeWindow("TrackedBars", 640, 240)

        hue_margin_default = 40

        cv2.createTrackbar("Hue Centre", "TrackedBars", hue, 179, nothing)
        cv2.createTrackbar("Hue Margin", "TrackedBars", hue_margin_default, 90, nothing)
        cv2.createTrackbar("Sat Min", "TrackedBars", 50, 255, nothing)
        cv2.createTrackbar("Sat Max", "TrackedBars", 255, 255, nothing)
        cv2.createTrackbar("Val Min", "TrackedBars", 128, 255, nothing)
        cv2.createTrackbar("Val Max", "TrackedBars", 50, 255, nothing)

        image, img = self.get_gripper_image()

        points = []
        while True:
            # Apply mask, with values given by trackbar
            lower, upper = get_trackbar()
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            if lower[0] > upper[0]:
                lower1 = lower.copy()
                upper1 = upper.copy()
                upper1[0] = 179
                mask1 = cv2.inRange(hsv, lower1, upper1)

                lower2 = lower.copy()
                lower2[0] = 0
                upper2 = upper.copy()
                mask2 = cv2.inRange(hsv, lower2, upper2)

                mask = cv2.bitwise_or(mask1, mask2)
                result = cv2.bitwise_and(img, img, mask=mask)
            else:
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
                    cv2.circle(result, (cx, cy), 7, (0, 0, 255), -1)
                    points.append([cx, cy])

            cv2.drawContours(result, contours, -1, (0,255,0), 3)
            cv2.imshow("TrackedBars", result)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('a'):
                break
            if key == ord('q'):
                points.clear()
                break

        print("Found {} points".format(len(points)))
        if len(points) == 0:
            return image, None

        point = points[0]
        image_pos = geometry_pb2.Vec2(x=point[0], y=point[1])

        return image, image_pos

    def get_object_position(self, image, target_shape, target_colour):
        resized_images, ratio = resize([image])
        print("image ratio:", ratio)
        masks_and_shapes = filter_all_colours(resized_images)
        shapes = detect_shapes(masks_and_shapes, ratio)
        mask, _ = masks_and_shapes[0]

        backtorgb = cv2.cvtColor(mask,cv2.COLOR_GRAY2RGB)
        cv2.imshow("Mask", mask)
        cv2.waitKey(0)
        print(shapes)
        for shape in shapes:
            if shape.shape_type == target_shape and shape.colour == target_colour:
                print("Object found")
                cv2.drawContours(backtorgb, shape.contour, -1, (20,255,0), 3)
                cv2.putText(backtorgb, shape.shape_type + " " + shape.colour, (shape.cX, shape.cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                # show the output image
                cv2.imshow("Object Image", backtorgb)
                cv2.waitKey(0)
                point = [shape.cX*ratio, shape.cY*ratio]
                image_pos = geometry_pb2.Vec2(x=point[0], y=point[1])
                return image_pos

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

        pose1 = math_helpers.SE3Pose(x=0.8, y=0, z=0.3, rot=initial_pose.rot)

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

        print("Sending trajectory command")

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

    def move_body_back(self, frame):
        print("Starting move backward")
        frame_yaw = 2*np.arccos(frame.rot.w)
        robot_cmd = RobotCommandBuilder.synchro_se2_trajectory_point_command(
            goal_x = frame.x,
            goal_y = frame.y,
            goal_heading = frame_yaw,
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
                break
        print("Finished moving backward")

    def drop_ball(self):
        robot_command = RobotCommandBuilder.claw_gripper_open_fraction_command(1.0)
        print("Sending gripper open command")
        cmd_id = self.command_client.robot_command(robot_command)

        # Wait until the arm arrives at the goal.
        #while True:
        #    feedback_resp = self.command_client.robot_command_feedback(cmd_id)
        #    if feedback_resp.feedback.synchronized_feedback.arm_command_feedback.arm_cartesian_feedback.status == arm_command_pb2.ArmCartesianCommand.Feedback.STATUS_TRAJECTORY_COMPLETE:
        #        break
        #    time.sleep(0.1)
        time.sleep(1)
        print("Finished opening gripper")

    def ask_for_shape_and_colour(self):
        shapes = ["ball", "skittle", "ring", "cone"]
        colours = ["red", "yellow", "blue", "purple", "green"]
        print("0: ball")
        print("1: skittle")
        print("2: ring")
        print("3: cone")
        shape_index = int(input("Choose a shape: "))

        shape = shapes[shape_index]

        print("0: red")
        print("1: yellow")
        print("2: blue")
        print("3: purple")
        print("4: green")
        colour_index = int(input("Choose a colour: "))
        colour = colours[colour_index]
        return (shape, colour)


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
            target_shape, target_colour = self.ask_for_shape_and_colour()
            
            
            image, img = self.get_gripper_image()
            point = self.get_object_position(img, target_shape, target_colour)
            success = self.grasp_ball(image, point)

            if success:
                self.move_arm_up(initial_flat_body_transform)
                self.move_body_back(initial_flat_body_transform)
                self.drop_ball()
            else:
                print("Failed to pickup ball, trying again")
                iter_num-=1

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
        scene_pos=[1, 0, 0],)

    robot.run(options)

if __name__ == '__main__':
    if not main(sys.argv[1:]):
        sys.exit(1)
