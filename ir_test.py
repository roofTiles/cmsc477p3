import robomaster
from robomaster import robot
import time

import numpy as np
from robomaster import robot
import time
import math

global dist

# Defines functionality that is specific
# to the robot handing off the lego tower (the giver)

# rotates until the heading of the lego
# is towards the robot
def search_lego(rotational_speed = 20, k = 0.01, ep_camera=None): 
    
    distance = 1
    print('GIVER: Seaching for Legos')

    # want bounding box as close to center of img in horizontal dir
    while np.abs(distance) > 50: # bounding box x-center must be 200 away from center of img

        if True: # if lego in FOV
            
            ep_chassis.drive_speed(x=0, y=0, z=k * 1 * rotational_speed, timeout=5)

        else:
            ep_chassis.drive_speed(x=0, y=0, z=rotational_speed, timeout=5)

        time.sleep(0.1)

    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5) # stop rotating
    print('GIVER: Facing the Legos')

# tell robot to move towards the lego tower
# k_t is translational proportional controller
# k_r is rotational proportional controller
# WORK IN PROGRESS
def move_to_lego(translation_speed = 0.20, rotational_speed = 10, 
                 k_t = 0.01/2, k_r = 0.01, ep_sensor=None):

    horizontal_distance = 0
    lego_dist = 100000
    goal_lego_dist = 30 # cm
    looking_down = False
    looking_down_2 = False

    print('GIVER: Moving towards the Legos')

    while (np.abs(lego_dist - goal_lego_dist) > 5):

        ep_sensor.sub_distance(freq=5, callback=sub_data_handler)
        time.sleep(0.1)
           

        if True: # if lego in FOV
            
            lego_dist = dist
            distance_error = 0 - lego_dist # finding error in vertical

            print(lego_dist)


            if (horizontal_distance <= 5):
                ep_chassis.drive_speed(x=-1*translation_speed * k_t * distance_error, y=0,
                                z=0, timeout=5)
                
            if (lego_dist < 60) and not looking_down:
                print("down1")
                looking_down = True

            elif (lego_dist < 45) and not looking_down_2:
                print("down2")
                looking_down_2 = True

            elif (lego_dist < 35 and (looking_down_2)):
                print("close")
                print("GIVER: MOVING TOWARDS LEGO TOWER, NOT USING CAMERA ANYMORE")
                speed = 0.065
                #ep_chassis.drive_speed(x=speed, y=0, z=0) # drive towards lego
                #time.sleep(./speed)
                ep_chassis.drive_speed(x=0, y=0, z=0)
                time.sleep(0.1)
                return

        else:
            ep_chassis.drive_speed(x=translation_speed, y=0,
                                z=0, timeout=5)
            
            time.sleep(0.1)
        ep_sensor.sub_distance(freq=5, callback=sub_data_handler)
    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5)

        
def sub_data_handler(sub_info):
    distance = sub_info
    print("tof1:{0}  tof2:{1}  tof3:{2}  tof4:{3}".format(distance[0], distance[1], distance[2], distance[3]))
    dist = distance[0]


if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_chassis = ep_robot.chassis
    ep_camera = ep_robot.camera
    ep_gripper = ep_robot.gripper
    ep_arm = ep_robot.robotic_arm

    ep_sensor = ep_robot.sensor

    ep_chassis.drive_speed(x=1)
    ep_sensor.sub_distance(freq=5, callback=sub_data_handler)
    time.sleep(1)
    ep_sensor.unsub_distance()
    ep_chassis.drive_speed(x=0)

    move_to_lego(ep_sensor=ep_sensor)
    ep_robot.close()


