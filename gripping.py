import queue
import numpy as np
from scipy import interpolate
import matplotlib.pyplot as plt
from robomaster import robot

import cv2
import time
from robomaster import camera
import csv

ep_robot = robot.Robot()
ep_robot.initialize(conn_type="ap")
ep_chassis = ep_robot.chassis

def LookDown(ep_gripper=None, ep_arm=None, x=0, y=30, power=50):
    # Move forward xmm, drop down ymm
    ep_arm.move(x=x, y=-y).wait_for_completed()

def LookUp(ep_gripper=None, ep_arm=None, x=0, y=30, power=50):
    # Move forward xmm, drop down ymm
    ep_arm.move(x=x, y=y).wait_for_completed()

def GrabLego(ep_gripper=None, ep_arm=None, x=62, y=60, power=50):
    # open gripper
    ep_gripper.open(power=power)
    time.sleep(1)

    # Move forward xmm, drop down ymm
    ep_arm.move(x=x, y=-y).wait_for_completed()

    # close gripper
    ep_gripper.close(power=power)
    time.sleep(1)

    # Move backward xmm, move up ymm
    ep_arm.move(x=-x, y=y).wait_for_completed()

def DropLego(ep_gripper=None, ep_arm=None, x=60, y=100, power=50):
    # Move forward xmm, drop down ymm
    ep_arm.move(x=x, y=-y).wait_for_completed()

    # close gripper
    ep_gripper.open(power=power)
    time.sleep(1)

    # Move backward xmm, move up ymm
    ep_arm.move(x=-x, y=y).wait_for_completed()

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_gripper = ep_robot.gripper

    ep_arm = ep_robot.robotic_arm

    GrabLego(ep_gripper=ep_gripper, ep_arm=ep_arm)
    DropLego(ep_gripper=ep_gripper, ep_arm=ep_arm)

    ep_robot.close()

