import robomaster
from robomaster import robot
import time

import numpy as np
from robomaster import robot
import time
import math

# Defines functionality that is specific
# to the robot handing off the lego tower (the giver)

# rotates until the heading of the lego
# is towards the robot
class DistanceTest():

    def __init__(self):
        super().__init__('distancetest')
        self.dist = 0

    def move_to_lego(self, translation_speed = 0.20, rotational_speed = 10, 
                    k_t = 0.01/2, k_r = 0.01, ep_sensor=None, ep_chassis=None):

        horizontal_distance = 0
        lego_dist = 100000
        goal_lego_dist = 30 # cm
        looking_down = False
        looking_down_2 = False

        print('GIVER: Moving towards the Legos')

        while (np.abs(lego_dist - goal_lego_dist) > 5):
            

            if True: # if lego in FOV
                
                lego_dist = self.dist
                print(lego_dist)
                    
                if (lego_dist > 60):
                    ep_chassis.drive_speed(x=0.5)

                else:
                    print("close")
                    ep_chassis.drive_speed(x=0, y=0, z=1, timeout=5)

                
                time.sleep(0.1)
        ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5)

            
    def sub_data_handler(sub_info, self):
        distance = sub_info
        print("tof1:{0}  tof2:{1}  tof3:{2}  tof4:{3}".format(distance[0], distance[1], distance[2], distance[3]))
        self.dist = distance[0]


if __name__ == '__main__':
        distspinner = DistanceTest()
        ep_robot = robot.Robot()
        ep_robot.initialize(conn_type="ap")
        ep_chassis = ep_robot.chassis
        ep_camera = ep_robot.camera
        ep_gripper = ep_robot.gripper
        ep_arm = ep_robot.robotic_arm

        ep_sensor = ep_robot.sensor

        ep_sensor.sub_distance(freq=5, callback=distspinner.sub_data_handler)
        distspinner.move_to_lego(ep_sensor=ep_sensor)
        
        ep_robot.close()


