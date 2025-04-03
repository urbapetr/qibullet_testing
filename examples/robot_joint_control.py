#!/usr/bin/env python
# coding: utf-8

import sys
import time
import pybullet as p
from qibullet import SimulationManager
from qibullet import PepperVirtual
from qibullet import NaoVirtual
from qibullet import RomeoVirtual

if __name__ == "__main__":
    simulation_manager = SimulationManager()

    if (sys.version_info > (3, 0)):
        rob = input("Which robot should be spawned? (pepper/nao/romeo): ")
    else:
        rob = raw_input("Which robot should be spawned? (pepper/nao/romeo): ")

    # Auto stepping set to False, the user has to manually step the simulation
    client = simulation_manager.launchSimulation(gui=True, auto_step=False)

    if rob.lower() == "nao":
        robot = simulation_manager.spawnNao(client, spawn_ground_plane=True)
    elif rob.lower() == "pepper":
        robot = simulation_manager.spawnPepper(client, spawn_ground_plane=True)
    elif rob.lower() == "romeo":
        robot = simulation_manager.spawnRomeo(client, spawn_ground_plane=True)
    else:
        print("You have to specify a robot, pepper, nao or romeo.")
        simulation_manager.stopSimulation(client)
        sys.exit(1)

    time.sleep(1.0)
    joint_parameters = list()

    for name, joint in robot.joint_dict.items():
        if "Finger" not in name and "Thumb" not in name:
            joint_parameters.append((
                p.addUserDebugParameter(
                    name,
                    joint.getLowerLimit(),
                    joint.getUpperLimit(),
                    robot.getAnglesPosition(name)),
                name))

    try:
        while True:

            for joint_parameter in joint_parameters:
                robot.setAngles(
                    joint_parameter[1],
                    p.readUserDebugParameter(
                        joint_parameter[0]), 1.0)

            # Step the simulation
            simulation_manager.stepSimulation(client)

    except KeyboardInterrupt:
        pass
    finally:
        simulation_manager.stopSimulation(client)

    try:
        while True:
            for joint_parameter in joint_parameters:
                robot.setAngles(
                    joint_parameter[1],
                    p.readUserDebugParameter(joint_parameter[0]), 1.0)

            # Step the simulation
            simulation_manager.stepSimulation(client)

    except KeyboardInterrupt:
        pass
    finally:
        simulation_manager.stopSimulation(client)


class Morse:
    pass


def choose_word():
    return "ahojda"


def create_dict(word):
    return {}


with Morse() as simu:
    letters = ["a", "b", "c", "d", "e", "f", "g", "h", "i", "j", "k", "l", "m",
               "n", "o", "p", "q", "r", "s", "t", "u", "v", "w", "x", "y", "z"]
    mistakes = 0
    word = choose_word()
    word_list = ["_" for _ in word]
    word_dict = create_dict(word)
    while True:
        print(" ".join(word_list))
        if mistakes >= 10:
            print("Konec hry, prohral jsi, slovo bylo: " + word)
            break
        if "_" not in word_list:
            print("Konec hry, vyhral jsi!")
            break
        letter = input("Hadej pismeno:")
        if letter in letters:
            letters.remove(letter)
            if letter.upper() in word:
                print("Pismeno " + letter.upper() + " se ve slove nachazi.")
                for i in word_dict[letter.upper()]:
                    word_list[i] = letter.upper()
            else:
                print("Pismeno " + letter.upper() + " se ve slove NEnachazi.")
                mistakes += 1
        else:
            print("Pismeno " + letter.upper() + " bylo uz hadano.")
            continue
