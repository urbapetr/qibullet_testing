#!/usr/bin/env python
# coding: utf-8
import math

import cv2
import time
import pybullet
import pybullet_data
from qibullet import Camera
from qibullet import SimulationManager
from qibullet import PepperVirtual

# Define the target position for Pepper to walk around
target_position = [1, 0, 0.5]
rotation_angle_radians = math.radians(1.0)

def go_forward_until_last_senzor_catches(pepper):
    while pepper.getRightLaserValue()[-1] == 3:
        pepper.moveTo(1, 0, 0)
        cv2.waitKey(1)
    return

def rotate_right_until_last_senzor_catches(pepper):
    while pepper.getRightLaserValue()[0] == 3:
        pepper.moveTo(0, 0, (-2) * math.pi / 10)
        cv2.waitKey(1)

def move_around(pepper, pepper_position):
    odchylka = [0]
    threshold = 0.2
    # spin until first detected
    spin_left(pepper)
    while (True):
        # move until first is not detected
        move_forward(pepper)
        # spin until first detected
        spin_right(pepper)
        current_position = pepper.getPosition()
        #print("Pepper start Position:", pepper_position)
        #print("Pepper end Orientation:", current_position)
        if (abs(abs(pepper_position[0]) - abs(current_position[0])) < threshold and
            abs(abs(pepper_position[1]) - abs(current_position[1])) < threshold):
            print("FINISHED?")

def spin_left(pepper):
    while(pepper.getRightLaserValue()[0] == 3):
        pepper.moveTo(0, 0, 0.1)

def spin_right(pepper):
    while (pepper.getRightLaserValue()[0] == 3 or pepper.getRightLaserValue()[len(pepper.getRightLaserValue())//2] != 3):
        pepper.moveTo(0, 0, -0.1)

def move_forward(pepper):
    while (pepper.getRightLaserValue()[0] != 3):
        pepper.moveTo(0.1, 0, 0)

def calculate_distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)


def main():
    simulation_manager = SimulationManager()
    client = simulation_manager.launchSimulation(gui=True)
    pepper = simulation_manager\
        .spawnPepper(client, spawn_ground_plane=True)

    pepper.goToPosture("Crouch", 0.6)
    time.sleep(1)
    pepper.goToPosture("Stand", 0.6)
    time.sleep(1)
    pepper.goToPosture("StandZero", 0.6)
    time.sleep(1)

    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())

    duck = pybullet.loadURDF(
        "duck_vhacd.urdf",
        basePosition=[5, 4, 0.5],
        globalScaling=10.0,
        physicsClientId=client)

    pepper.showLaser(True)
    pepper.subscribeLaser()

    # Subscribe to the top RGB camera in QVGA (default value, specified here
    # for more clarity) and 15fps
    handle_top = pepper.subscribeCamera(
        PepperVirtual.ID_CAMERA_TOP,
        resolution=Camera.K_QVGA,
        fps=15.0)

    # Same process for the bottom camera
    handle_bottom = pepper.subscribeCamera(
        PepperVirtual.ID_CAMERA_BOTTOM,
        resolution=Camera.K_QVGA,
        fps=15.0)

    joint_parameters = list()

    for name, joint in pepper.joint_dict.items():
        if "Finger" not in name and "Thumb" not in name:
            joint_parameters.append((
                pybullet.addUserDebugParameter(
                    name,
                    joint.getLowerLimit(),
                    joint.getUpperLimit(),
                    pepper.getAnglesPosition(name)),
                name))

    object_position, object_orientation = pybullet.getBasePositionAndOrientation(duck)
    print("Object Position:", object_position)
    print("Object Orientation:", object_orientation)
    desired_heading_angle = math.atan2(object_position[1], object_position[0])
    pepper.moveTo(20, 0, 0)

    try:
        while True:
            for joint_parameter in joint_parameters:
                pepper.setAngles(
                    joint_parameter[1],
                    pybullet.readUserDebugParameter(
                        joint_parameter[0]), 1.0)

            img_top = pepper.getCameraFrame(handle_top)
            img_bottom = pepper.getCameraFrame(handle_bottom)
            cv2.imshow("top camera", img_top)
            cv2.imshow("bottom camera", img_bottom)
            # Step the simulation
            simulation_manager.stepSimulation(client)
            cv2.waitKey(1)

            # Read laser scan data
            laser_data = pepper.getFrontLaserValue()

            # Process laser data to detect obstacles
            obstacle_detected = False
            for distance in laser_data:
                if distance < 1:
                    obstacle_detected = True
                    break

            if obstacle_detected:
                # Plan a new path to avoid the obstacle
                # You can use path planning or simple navigation commands here
                # Adjust the angle and walking behavior accordingly

                # For example, make Pepper rotate to the left
                pepper_position = pepper.getPosition()
                move_around(pepper, pepper_position)

            else:
                # No obstacle detected, move towards the target
                #pepper.moveTo(object_position[0]/move_by, 0, 0)
                #move_by += 1
                distance = calculate_distance(pepper.getPosition()[0], pepper.getPosition()[1], object_position[0], object_position[1])
                pepper.moveTo(distance - 1, 0, 0)
                simulation_manager.stepSimulation(client)

            # Step the simulation
            simulation_manager.stepSimulation(client)

    except KeyboardInterrupt:
        pass
    finally:
        # No need to manually unsubscribe from the cameras when calling
        # stopSimulation, the method will automatically unsubscribe from the
        # cameras
        simulation_manager.stopSimulation(client)

def main2():
    # Set up the simulation
    simulation_manager = SimulationManager()
    client = simulation_manager.launchSimulation(gui=True)
    pepper = simulation_manager.spawnPepper(client, spawn_ground_plane=True)

    # Load the object you want Pepper to interact with
    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
    print(pybullet_data.getDataPath())

    table_id = pybullet.loadURDF(
        "table.urdf",
        basePosition=[0, -2, 0],
        globalScaling=1.0,
        physicsClientId=client,
        flags=pybullet.URDF_USE_SELF_COLLISION)

    duck_id = pybullet.loadURDF(
        "duck_vhacd.urdf",
        basePosition=[0, -1.6, 1],
        globalScaling=1.0,
        physicsClientId=client,
        flags=pybullet.URDF_USE_SELF_COLLISION)

    # Otočení Peppera
    object_position, object_orientation = pybullet.getBasePositionAndOrientation(duck_id)
    desired_heading_angle = math.atan2(object_position[1], object_position[0] - 0.1)
    pepper.moveTo(0, 0, desired_heading_angle)

    # Pohyb blíže k předmětu
    distance = calculate_distance(pepper.getPosition()[0], pepper.getPosition()[1], object_position[0]-0.1,
                                  object_position[1])
    pepper.moveTo(distance - 0.25, 0, 0)

    # Zavolání funkce pro zvednutí předmětu
    lift_object(pepper, object_position, simulation_manager, client)

    simulation_manager.stopSimulation(client)


def lift_object(pepper, object_position, simulation_manager, client):
    # Otevření levé ruky
    pepper.setAngles('LHand', 1.0, 1.0)
    time.sleep(1)
    simulation_manager.stepSimulation(client)

    # Pohyb ruky blíže k předmětu
    pepper.setAngles('LShoulderPitch', object_position[2] + 0.3, 0.2)
    time.sleep(1)
    simulation_manager.stepSimulation(client)

    # Zavření levé ruky pro simulaci chycení předmětu
    pepper.setAngles('LHand', 0.0, 1.0)
    time.sleep(1)
    simulation_manager.stepSimulation(client)

    # Zvednutí předmětu
    pepper.setAngles('LShoulderPitch', 0.0, 0.5)
    time.sleep(1)
    simulation_manager.stepSimulation(client)

    # Puštění předmětu
    pepper.setAngles('LHand', 1.0, 1.0)
    time.sleep(1)
    simulation_manager.stepSimulation(client)

    """
    # Assuming the hand joints of Pepper are named 'LHand' and 'RHand'
    left_hand_joint = pepper.joint_dict['LHand']
    right_hand_joint = pepper.joint_dict['RHand']

    # Open the hands
    pepper.setAngles('LHand', 1.0, 1.0)
    pepper.setAngles('RHand', 1.0, 1.0)
    pepper.setAngles('HipPitch', -0.1, 1.0)

    # Close the hands to simulate touching the object
    pepper.setAngles('HipPitch', 0.0, 1.0)"""


    """
    joint_angles = pepper.getAnglesPosition("LShoulderPitch")
    target_position = pepper.getLinkPosition("LShoulderPitch", "LHand")

    # Adjust the target position based on the object's position
    target_position[0] = object_position[0]
    target_position[1] = object_position[1]
    target_position[2] = object_position[2] + 0.1  # Lift the gripper above the object

    joint_angles = pepper.getIK("LShoulderPitch", target_position)

    # Move to the calculated joint angles
    pepper.setAngles("LShoulderPitch", joint_angles, 1.0)

    # Grasp the object (simulated)
    pepper.setAngles("LHand", 0.5, 1.0)

    # Lift the object
    pepper.setAngles("LShoulderPitch", [joint + 0.1 for joint in joint_angles], 1.0)

    time.sleep(5)  # Simulate holding the object for 5 seconds

    pepper.setAngles("LHand", 0.0, 1.0)"""

if __name__ == "__main__":
    main2()
