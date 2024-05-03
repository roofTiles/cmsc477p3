import numpy as np
from robomaster import robot
from robomaster import camera
from matplotlib import pyplot as plt
import time
import math 
import detection
import gripping
import cv2
# import messagingserver

# Defines functionality that is specific
# to the robot handing off the lego tower (the giver)

# rotates until the heading of the lego
# is towards the robot
def search_lego(rotational_speed = 20, k = 0.01, ep_camera=None): 
    
    distance = 1000000
    print('GIVER: Seaching for Legos')

    # want bounding box as close to center of img in horizontal dir
    while np.abs(distance) > 50: # bounding box x-center must be 200 away from center of img

        results = detection.detect_object_in_image('lego', ep_camera=ep_camera, conf=0.5)

        if results[0] > 0: # if lego in FOV
            # ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5)
            # time.sleep(3)
            bb = results[1] # bounding box -  array of format [x,y,w,h] scaled to image size of (384, 640)
            horizontal_center = bb[0]
            distance = horizontal_center - 640/2 # finding error in horizontal
            ep_chassis.drive_speed(x=0, y=0, z=k * distance * rotational_speed, timeout=5)

        else:
            ep_chassis.drive_speed(x=0, y=0, z=rotational_speed, timeout=5)

        # time.sleep(0.1)

    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5) # stop rotating
    print('GIVER: Facing the Legos')

# tell robot to move towards the lego tower
# k_t is translational proportional controller
# k_r is rotational proportional controller

def move_to_lego(translation_speed = 0.20, rotational_speed = 10, 
                 k_t = 0.01/2, k_r = 0.01, ep_camera=None):

    horizontal_distance = 1000000
    lego_dist = 100000
    goal_lego_dist = 25 # cm
    looking_down = False
    looking_down_2 = False

    print('GIVER: Moving towards the Legos')

    while (np.abs(lego_dist - goal_lego_dist) > 5):
           
        results = detection.detect_object_in_image('lego', ep_camera=ep_camera, conf=0.5)

        if results[0]: # if lego in FOV
            
            lego_dist = (384/bb[3] * 18.5)/2.0 * 1/math.tan(50/180.0 * math.pi) # gives distance to lego in cm
            horizontal_center = bb[0]
            distance_error = 0 - lego_dist # finding error in vertical
            horizontal_distance = horizontal_center - 320 # finding error in horizontal

            if (horizontal_distance > 5):
                ep_chassis.drive_speed(x=-1*translation_speed * k_t * distance_error, y=0,
                                z= rotational_speed * k_r * horizontal_distance, timeout=5)

            if (horizontal_distance <= 5):
                ep_chassis.drive_speed(x=-1*translation_speed * k_t * distance_error, y=0,
                                z=0, timeout=5)
                
            if (lego_dist >= 35 and bb[1] <= 280):
                print('too far')
            
            elif (lego_dist < 60 or bb[1] > 210) and not looking_down:
                gripping.LookDown(ep_arm=ep_arm)
                looking_down = True

            elif (lego_dist < 45 or bb[1] > 210) and not looking_down_2:
                gripping.LookDown(ep_arm=ep_arm)
                looking_down_2 = True

            elif (lego_dist < 35 and (bb[1] > 280 and looking_down_2)): # when too close to lego tower
                print("GIVER: MOVING TOWARDS LEGO TOWER, NOT USING CAMERA ANYMORE")
                speed = 0.065
                ep_chassis.drive_speed(x=0, y=0, z=0)
                time.sleep(0.1)
                return
            
            else:
                ep_chassis.drive_speed(x=0, y=0, z=0)
                time.sleep(0.1)
                return
            
        else:
            ep_chassis.drive_speed(x=translation_speed, y=0,
                                z=0, timeout=5)
            
            time.sleep(0.1)
    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5)

def move_to_line(ep_camera=None):

    Oriented = False

    while not Oriented:
        # Input camera feed
        image = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)

        # apply gaussian blur
        Gaussian = cv2.GaussianBlur(image, (13, 9), 0)

        hsv = cv2.cvtColor(Gaussian, cv2.COLOR_BGR2HSV)

        # Threshold of blue in HSV space
        lower_blue = np.array([60, 60, 130])  # [60, 35, 140]
        upper_blue = np.array([160, 220, 255])  # [180, 255, 255]

        # preparing the mask to overlay
        mask2 = cv2.inRange(hsv, lower_blue, upper_blue)

        # The black region in the mask has the value of 0,
        # so when multiplied with original image removes all non-blue regions
        result = cv2.bitwise_and(Gaussian, Gaussian, mask=mask2)

        # Apply edge detection method on the image
        edges = cv2.Canny(result, 50, 150, apertureSize=3)

        # This returns an array of r and theta values
        lines = cv2.HoughLines(edges, 1, np.pi / 180, 200)
        r_vals = 0
        thet_vals = 0


        if not lines is None:

            # average all lines
            for i in range(len(lines)):
                r_vals = r_vals + lines[i][0][0]
                thet_vals = thet_vals + lines[i][0][1]
                r = r_vals / (i + 1)
                thet = thet_vals / (i + 1)

                # yaw angle
                yaw = (math.pi / 2) - thet 
                print("yaw", np.rad2deg(yaw))
                
            y = []
            x = []

            # determine equation of mean line to plot
            if thet < math.pi / 2:
                x0 = r / math.cos(thet)
                y0 = r / math.cos(math.pi / 2 - thet)
                m = -y0 / x0

            elif thet > math.pi / 2:
                x0 = -r / math.cos(math.pi - thet)
                y0 = r / math.cos(thet - math.pi / 2)
                m = -y0 / x0
            

            for j in range(0, 1280):
                if m * j + y0 < 720:
                    x.append(j)
                    y.append(m * j + y0)
            
            
            
            # orient robot
            ep_chassis.drive_speed(x=0, y=0, z=-1*np.rad2deg(yaw))
            time.sleep(1)
            
            if np.rad2deg(yaw) > -3 and np.rad2deg(yaw) < 3:
                Oriented = True
                
                # move robot
                y_min = 110*2  # pixels 124
                y_end = 285*2  # pixels 285
                ws_inside_dist = 1.15  # [m]
                scale = ws_inside_dist/(y_end - y_min) # conversion from pix to meters
                if len(y) > 0:
                    pixel_dist = round((y[0] + y[-1])/2)
                else:
                    pixel_dist = y_end
                x_vel = abs(pixel_dist-y_end)*scale

                ep_chassis.drive_speed(x=x_vel/11, y=0, z=0)
                time.sleep(10)
                ep_chassis.drive_speed(x=0, y=0, z=0)
                time.sleep(0.5)
            
        else:
            print('Target line is out of view')
            ep_chassis.drive_speed(x=0, y=0, z=-15)
            time.sleep(1)

        

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_chassis = ep_robot.chassis
    ep_camera = ep_robot.camera
    ep_gripper = ep_robot.gripper
    ep_arm = ep_robot.robotic_arm
    
    ep_camera.start_video_stream(display=True)
    
    ep_gripper.open(power=50) # for grabbing

    # search for the lego
    search_lego(ep_camera=ep_camera)
    
    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5)
    time.sleep(0.5)

    # move to lego
    move_to_lego(ep_camera=ep_camera)
    
    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5)
    time.sleep(0.5)

    # grab lego
    gripping.GrabLego(ep_gripper=ep_gripper, ep_arm=ep_arm)
    print('Done grabbing, looking down for line')

    # move to line
    move_to_line(ep_camera=ep_camera)
    print('done moving to line')

    time.sleep(30) # delay for receiver to come to giver

    print('releasing gripper')
    ep_gripper.open(power=50)

    # go backwards when done gripping
    ep_chassis.drive_speed(x=-0.2, y=0, z=0, timeout=5)
    time.sleep(1.5)
    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5)
    time.sleep(0.1)

    ep_robot.close()
