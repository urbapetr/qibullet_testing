#!/usr/bin/env python
# coding: utf-8

import time
import pybullet
import pybullet_data
from qibullet import PepperVirtual
from qibullet import SimulationManager


def main():
    simulation_manager = SimulationManager()
    client = simulation_manager.launchSimulation(gui=True)
    pepper = simulation_manager.spawnPepper(client, spawn_ground_plane=True)

    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())

    pybullet.loadURDF(
        "duck_vhacd.urdf",
        basePosition=[1, 0, 0.5],
        globalScaling=10.0,
        physicsClientId=client)

    pepper.showLaser(True)
    pepper.subscribeLaser()
    pepper.goToPosture("Stand", 0.6)

    try:
        while True:
            laser_list = pepper.getRightLaserValue()
            laser_list.extend(pepper.getFrontLaserValue())
            laser_list.extend(pepper.getLeftLaserValue())

            if all(laser == 3.0 for laser in pepper.getRightLaserValue()):
                print("RIGHT: Nothing detected")
            else:
                print("RIGHT: Detected")
                pass

            if all(laser == 3.0 for laser in pepper.getFrontLaserValue()):
                print("FRONT: Nothing detected")
            else:
                print("FRONT: Detected")
                pass

            if all(laser == 3.0 for laser in pepper.getLeftLaserValue()):
                print("LEFT: Nothing detected")
            else:
                print("LEFT: Detected")
                pass

            if all(laser == 3.0 for laser in laser_list):
                print("Nothing detected")
            else:
                print("Detected")
                pass

            time.sleep(0.1)

    except KeyboardInterrupt:
        pass
    finally:
        simulation_manager.stopSimulation(client)


if __name__ == "__main__":
    main()
