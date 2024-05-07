import robomaster
from robomaster import robot
import time

import numpy as np
from robomaster import robot
import time
import math

# if robot starts off on the left side this is
# -1 otherwise if it is the right side this is 1
y_sign = -1

class DistanceTest():

    def __init__(self, ep_chassis, ep_sensor):
        self.dist = 0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.ep_chassis = ep_chassis
        self.ep_sensor = ep_sensor

    def avoid(self, translation_speed = 0.20, rotational_speed = 10, 
                    k_t = 0.01/2, k_r = 0.01):

        horizontal_distance = 0
        obj_dist = 100000
        x_speed = 0.2

        threshold_dist = 300
        camera_angle = math.atan(threshold_dist/100)
        
        while self.x < 2.0:
            obj_dist = math.sin(camera_angle) * self.dist
            print("x: ", self.x, "y: ", self.y, "Object: ", obj_dist)

            if self.y > 2.0:
                ep_chassis.drive_speed(y=0.2)
                time.sleep(1)

            elif self.y < -0.3:
                ep_chassis.drive_speed(y=-0.2)
                time.sleep(1)
                    
            elif (obj_dist > threshold_dist):
                ep_chassis.drive_speed(x=x_speed)
                time.sleep(1)

            else:
                print("close to obj")

                if (self.y <= 2.5/2): # prevent veering off
                    ep_chassis.drive_speed(x=0, y=0.3, z=0, timeout=5)
                    time.sleep(0.5)

                elif (self.y > 2.5/2):
                    ep_chassis.drive_speed(x=0, y=-0.3, z=0, timeout=5)
                    time.sleep(0.5)

        ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5)
        time.sleep(0.1)

def sub_data_handler(sub_info):
    distance = sub_info
    distspinner.dist = distance[0]

def sub_position_handler(position_info):
    x, y, z = position_info
    distspinner.x = y
    distspinner.y = -x
    distspinner.z = z


if __name__ == '__main__':
        
        ep_robot = robot.Robot()
        ep_robot.initialize(conn_type="ap")
        ep_chassis = ep_robot.chassis
        ep_camera = ep_robot.camera
        ep_gripper = ep_robot.gripper
        ep_arm = ep_robot.robotic_arm

        ep_sensor = ep_robot.sensor

        distspinner = DistanceTest(ep_chassis=ep_chassis, ep_sensor=ep_sensor)
        ep_sensor.sub_distance(freq=10, callback=sub_data_handler)
        ep_chassis.sub_position(freq=10, callback=sub_position_handler)

        time.sleep(1)

        distspinner.avoid()

        ep_sensor.unsub_distance()
        ep_chassis.unsub_position()
        ep_robot.close()


