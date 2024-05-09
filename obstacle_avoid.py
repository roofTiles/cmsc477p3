import detection
import robomaster
from robomaster import robot
import time

import numpy as np
from robomaster import robot
import time
import math
import cv2


class DistanceTest():

    def __init__(self, ep_chassis, ep_sensor):
        self.dist = 0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.ep_chassis = ep_chassis
        self.ep_sensor = ep_sensor
    
    # moves the robot towards the target lego as long as the lego is in the camera frame.
    # also moves robot side to side to avoid obstacles during approach.
    def move_robot(self, is_left, x_speed = 0.2, y_speed = 0.3, sleep_time = 0.1, h_obstacle_max = 340, threshold_dist = 400, ep_camera = None):
        print("GIVER: Moving to lego")

        # if robot starts off on the left side this is
        # -1 otherwise if it is the right side this is 1
        if is_left:
            sign = 1
        else: 
            sign = -1

        # parameter for camera pointing angle
        camera_angle = math.atan(threshold_dist/100)

        while self.x < 1:
            # use IR sensor to find distance to obstacle
            sensor_dist = math.sin(camera_angle) * self.dist
            print("x: ", self.x, "y: ", self.y, "Object: ", sensor_dist)

            # read in largest (nearest) YOLO bounding box for obstacles and lego
            bb_obstacle = detection.detect_object_in_image(c='obstacle', conf=0.35, image=None, ep_camera=ep_camera)

            # if obstacles are in view, do the following...
            if bb_obstacle[0] and ((sensor_dist < threshold_dist) or (bb_obstacle[1][2] > h_obstacle_max)):
                # if an obstacle is in view and its centroid is to the right of the camera center, slide robot left
                if (bb_obstacle[1][0] > 1280/2):
                    print("moving left")
                    ep_chassis.drive_speed(x=0, y=sign*-y_speed, z=0, timeout=5)
                    time.sleep(10*sleep_time)
                    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5)
                    time.sleep(2*sleep_time)
                # if an obstacle is in view and its centroid is to the left of the camera center, slide robot right
                else:
                    print("moving right")
                    ep_chassis.drive_speed(x=0, y=sign*y_speed, z=0, timeout=5)
                    time.sleep(10*sleep_time)
                    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5)
                    time.sleep(2*sleep_time)
                
                # # if an obstacle is in view and its centroid is to the right of the camera center, slide robot left
                # if (bb_obstacle[1][0] > 1280/2): 
                #     if self.y > 0.3:
                #         ep_chassis.drive_speed(x=0, y=-y_speed, z=0, timeout=5)
                #         time.sleep(20*sleep_time)
                #     else:
                #         ep_chassis.drive_speed(x=0, y=y_speed, z=0, timeout=5)
                #         time.sleep(20*sleep_time)
                # # if an obstacle is in view and its centroid is to the left of the camera center, slide robot right
                # else:
                #     if self.y > 0.3:
                #         ep_chassis.drive_speed(x=0, y=y_speed, z=0, timeout=5)
                #         time.sleep(20*sleep_time)
                #     else:
                #         ep_chassis.drive_speed(x=0, y=-y_speed, z=0, timeout=5)
                #         time.sleep(20*sleep_time)
                #     # ep_chassis.drive_speed(x=0, y=y_speed, z=0, timeout=5)
                #     # time.sleep(10*sleep_time)
            
            else:
                ep_chassis.drive_speed(x=x_speed, y=0, z=0, timeout=5)
                time.sleep(sleep_time)
        
        ep_chassis.drive_speed(x=0, y=0, z=sign*40, timeout=5)
        time.sleep(20*sleep_time)

        while self.x < 1.4:
            # use IR sensor to find distance to obstacle
            sensor_dist = math.sin(camera_angle) * self.dist
            print("x: ", self.x, "y: ", self.y, "Object: ", sensor_dist)

            # read in largest (nearest) YOLO bounding box for obstacles and lego
            bb_obstacle = detection.detect_object_in_image(c='obstacle', conf=0.35, image=None, ep_camera=ep_camera)

            # if obstacles are in view, do the following...
            if bb_obstacle[0] and ((sensor_dist < threshold_dist) or (bb_obstacle[1][2] > h_obstacle_max)):
                # if an obstacle is in view and its centroid is to the right of the camera center, slide robot left
                if (bb_obstacle[1][0] > 1280/2) and self.y :
                    print("moving left")
                    ep_chassis.drive_speed(x=0, y=sign*-y_speed, z=0, timeout=5)
                    time.sleep(10*sleep_time)
                    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5)
                    time.sleep(2*sleep_time)

                # if an obstacle is in view and its centroid is to the left of the camera center, slide robot right
                else:
                    print("moving right")
                    ep_chassis.drive_speed(x=0, y=sign*y_speed, z=0, timeout=5)
                    time.sleep(10*sleep_time)
                    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5)
                    time.sleep(2*sleep_time)
            
            else:
                ep_chassis.drive_speed(x=x_speed, y=0, z=0, timeout=5)
                time.sleep(sleep_time)
        return
    
    
    def move_to_line(self, x_speed = 0.1, ep_camera=None):

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
            print(str(lines))
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
                #time.sleep(0.1)
                
                if np.rad2deg(yaw) > -3 and np.rad2deg(yaw) < 3:                              
                    Oriented = True
                    # move robot
                    y_min = 110*2  # pixels 124
                    y_end = 285*2  # pixels 285
                    ws_inside_dist = 1.15  # [m]
                    scale = ws_inside_dist/(y_end - y_min)
                    if len(y) > 0:
                        pixel_dist = round((y[0] + y[-1])/2)
                    else:
                        pixel_dist = y_end
                    x_vel = abs(pixel_dist-y_end)*scale
                    print("pixel distance:", pixel_dist)
                    print("actual distance", x_vel)
                    ep_chassis.drive_speed(x=0.25, y=0, z=0) #x_vel
                    time.sleep(5)
                    ep_chassis.drive_speed(x=0, y=0, z=0)
                    time.sleep(0.5)
                
            else:
                print('Target line is out of view')
                ep_chassis.drive_speed(x=0, y=0, z=-15)
                time.sleep(1)

def sub_data_handler(sub_info):
    distance = sub_info
    distspinner.dist = distance[0]

def sub_position_handler(position_info):
    x, y, z = position_info

    # change as needed
    if y < 0:
        y = -y
    if x < 0:
        x = -x
    distspinner.x = y
    distspinner.y = x

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
        ep_camera.start_video_stream(display=True)

        time.sleep(1)
        ep_gripper.open(power=100)
        time.sleep(1)
        ep_arm.move(x=100, y=0).wait_for_completed()


        time.sleep(1)

        distspinner.move_robot(is_left=True, ep_camera=ep_camera)
        distspinner.move_to_line(ep_camera=ep_camera)

        ep_sensor.unsub_distance()
        ep_chassis.unsub_position()
        ep_robot.close()


