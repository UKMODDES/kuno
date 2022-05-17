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

# Copyright (c) 2022 Boston Dynamics, Inc.  All rights reserved.
#
# Downloading, reproducing, distributing or otherwise using the SDK Software
# is subject to the terms and conditions of the Boston Dynamics Software
# Development Kit License (20191101-BDSDK-SL).

# https://dev.bostondynamics.com/choreography_protos/bosdyn/api/choreography_reference

import os
import argparse
import sys
import time

import bosdyn.client
import bosdyn.client.util
from bosdyn.client import create_standard_sdk, RpcError, ResponseError
from bosdyn.client.exceptions import UnauthenticatedError
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.license import LicenseClient
from bosdyn.choreography.client.choreography import ChoreographyClient, load_choreography_sequence_from_txt_file

class Options:
    def __init__(self, hostname, username, password):
        self.hostname = hostname
        self.username = username
        self.password = password

def main(argv):
    options = Options("192.168.80.3", "Kuno", "olympickuno1")
    sdk = bosdyn.client.create_standard_sdk('UploadChoreography')
    sdk.register_service_client(ChoreographyClient)
    robot = sdk.create_robot(options.hostname)
    robot.authenticate(options.username, options.password)

    # Create estop
    estop_client = robot.ensure_client("estop")
    estop_endpoint = EstopEndpoint(client=estop_client, name="my_estop", estop_timeout=9.0)
    estop_endpoint.force_simple_setup()
    estop_keep_alive = EstopKeepAlive(estop_endpoint)

    license_client = robot.ensure_client(LicenseClient.default_service_name)
    if not license_client.get_feature_enabled([ChoreographyClient.license_name])[ChoreographyClient.license_name]:
        print("This robot is not licensed for choreography.")
        sys.exit(1)

    # Check that an estop is connected with the robot so that the robot commands can be executed.
    assert not robot.is_estopped(), "Robot is estopped. Please use an external E-Stop client, " \
                                    "such as the estop SDK example, to configure E-Stop."

    # Create a lease and lease keep-alive so we can issue commands. A lease is required to execute
    # a choreographed sequence.
    lease_client = robot.ensure_client(LeaseClient.default_service_name)
    lease = lease_client.take()
    lk = LeaseKeepAlive(lease_client)

    # Create the client for the Choreography service.
    choreography_client = robot.ensure_client(ChoreographyClient.default_service_name)

    choreography_filepath = "dance.csq" #"dance.csq"  // routine.csq
    try:
        choreography = load_choreography_sequence_from_txt_file("dance.csq")
        choreography = load_choreography_sequence_from_txt_file("routine.csq")
    except Exception as execp:
        print("Failed to load choreography. Raised exception: " + str(execp))
        return True

    # Once the choreography is loaded into a protobuf message, upload the routine to the robot. We set
    # non_strict_parsing to true so that the robot will automatically correct any errors it find in the routine.
    try:
        upload_response = choreography_client.upload_choreography(load_choreography_sequence_from_txt_file("dance.csq"),
                                                                  non_strict_parsing=True)
        upload_response = choreography_client.upload_choreography(load_choreography_sequence_from_txt_file("routine.csq"),
                                                                  non_strict_parsing=True)
    except UnauthenticatedError as err:
        print(
            "The robot license must contain 'choreography' permissions to upload and execute dances. "
            "Please contact Boston Dynamics Support to get the appropriate license file. ")
        return True
    except ResponseError as err:
        # Check if the ChoreographyService considers the uploaded routine as valid. If not, then the warnings must be
        # addressed before the routine can be executed on robot.
        error_msg = "Choreography sequence upload failed. The following warnings were produced: "
        for warn in err.response.warnings:
            error_msg += warn
        print(error_msg)
        return True

    sequences_on_robot = choreography_client.list_all_sequences()
    print('Sequence uploaded. All sequences on the robot:\n{}'.format('\n'.join(
        sequences_on_robot.known_sequences)))
    # if options.upload_only:
    #     return True

    # If the routine was valid, then we can now execute the routine on robot.
    # Power on the robot. The robot can start from any position, since the Choreography Service can automatically
    # figure out and move the robot to the position necessary for the first move.
    robot.power_on()

    # First, get the name of the choreographed sequence that was uploaded to the robot to uniquely identify which
    # routine to perform.
    # routine_name = choreography.name
    

    # ### Preloaded samples
    # Demo_Dance
    # Backward Anticipation
    # Demo_Dance
    # Forward Anticipation
    # Hallway Turn Left
    # Hallway Turn Right
    # HallwayPushBall
    # Kuno routine
    # Left Side Anticipation
    # Performance_Bow
    # Right Side Anticipation
    # Safe Stand
    # action
    # complete
    # lost



    # Then, set a start time five seconds after the current time.
    client_start_time = time.time() + 5.0
    # Specify the starting slice of the choreography. We will set this to slice=0 so that the routine begins at
    # the very beginning.
    start_slice = 0
    # Issue the command to the robot's choreography service.

    choreography_client.execute_choreography(choreography_name="lost",
                                             client_start_time=client_start_time,
                                             choreography_starting_slice=0)

    time.sleep(10)
    client_start_time = time.time() + 1.0
    choreography_client.execute_choreography(choreography_name="Forward Anticipation",
                                             client_start_time=client_start_time,
                                             choreography_starting_slice=0)

    time.sleep(1)
    client_start_time = time.time() + 1.0
    choreography_client.execute_choreography(choreography_name="Hallway Turn Left",
                                             client_start_time=client_start_time,
                                             choreography_starting_slice=0)

    time.sleep(5)
    client_start_time = time.time() + 1.0
    choreography_client.execute_choreography(choreography_name="Left Side Anticipation",
                                             client_start_time=client_start_time,
                                             choreography_starting_slice=0)

    time.sleep(2)
    client_start_time = time.time() + 1.0
    choreography_client.execute_choreography(choreography_name="Hallway Turn Right",
                                             client_start_time=client_start_time,
                                             choreography_starting_slice=0)

    time.sleep(5)
    client_start_time = time.time() + 1.0
    choreography_client.execute_choreography(choreography_name="Backward Anticipation",
                                             client_start_time=client_start_time,
                                             choreography_starting_slice=0)

    time.sleep(2)
    client_start_time = time.time() + 1.0
    choreography_client.execute_choreography(choreography_name="HallwayPushBall",
                                             client_start_time=client_start_time,
                                             choreography_starting_slice=0)

    time.sleep(5)
    client_start_time = time.time() + 1.0
    choreography_client.execute_choreography(choreography_name="Demo_Dance",
                                             client_start_time=client_start_time,
                                             choreography_starting_slice=0)

    time.sleep(10)
    client_start_time = time.time() + 2.0
    choreography_client.execute_choreography(choreography_name="Kuno routine",
                                             client_start_time=client_start_time,
                                             choreography_starting_slice=start_slice)

    time.sleep(20)
    client_start_time = time.time() + 2.0
    choreography_client.execute_choreography(choreography_name="Upload Choreography Example",
                                             client_start_time=client_start_time,
                                             choreography_starting_slice=start_slice)

    time.sleep(20)
    client_start_time = time.time() + 1.0
    choreography_client.execute_choreography(choreography_name="Performance_Bow",
                                             client_start_time=client_start_time,
                                             choreography_starting_slice=0)

    # client_start_time = time.time() + 2.0
    # choreography_client.execute_choreography(choreography_name=routine_name,
    #                                          client_start_time=client_start_time,
    #                                          choreography_starting_slice=start_slice)

    # Estimate how long the choreographed sequence will take, and sleep that long.
    total_choreography_slices = 0
    for move in choreography.moves:
        total_choreography_slices += move.requested_slices
    estimated_time_seconds = total_choreography_slices / choreography.slices_per_minute * 60.0
    time.sleep(estimated_time_seconds)

    # Sit the robot down and power off the robot.
    robot.power_off()
    return True


if __name__ == "__main__":
    if not main(sys.argv[1:]):
        sys.exit(1)
