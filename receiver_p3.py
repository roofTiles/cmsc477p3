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
        self.dist = 1000
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.ep_chassis = ep_chassis
        self.ep_sensor = ep_sensor
    
    # Rotate robot until lego is spotted. 
    # If no lego is spotted after a full rotation, translate slightly in some direction and repeat.
    def look_for_lego(self, x_speed = 0.2, z_speed = 10, sleep_time = 0.1, h_obstacle_max = 450, h_lego_max = 80, threshold_dist = 300, ep_camera = None):
        print("GIVER: Looking for lego")
        # parameter for camera pointing angle
        camera_angle = math.atan(threshold_dist/100) 

        while True:
            # use IR sensor to find distance to obstacle
            sensor_dist = math.sin(camera_angle) * self.dist
            print("Object distance: ", sensor_dist)

            # read in largest (nearest) YOLO bounding box for obstacles and lego
            bb_obstacle = detection.detect_object_in_image(c='obstacle', conf=0.35, image=None, ep_camera=ep_camera)
            bb_lego = detection.detect_object_in_image(c='lego', conf=0.35, image=None, ep_camera=ep_camera)
            rotation = 0

            # while no lego is found...
            while not bb_lego[0]:
                print(rotation)

                # continue to rotate robot if it has not rotated in a full circle or if there is an obstacle nearby.
                if (rotation < 360) or (bb_obstacle[1][3] > h_obstacle_max or sensor_dist < threshold_dist):
                    ep_chassis.drive_speed(x=0, y=0, z=z_speed, timeout=5)
                    time.sleep(sleep_time)
                    # ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5)
                    # time.sleep(sleep_time)
                    rotation += (z_speed*sleep_time)
                
                # otherwise, move the robot slightly forward in the direction it is pointing.
                else: 
                    ep_chassis.drive_speed(x=x_speed, y=0, z=0, timeout=5)
                    time.sleep(5*sleep_time)
                    rotation = 0
                
                # keep looking for lego
                bb_obstacle = detection.detect_object_in_image(c='obstacle', conf=0.35, image=None, ep_camera=ep_camera)
                bb_lego = detection.detect_object_in_image(c='lego', conf=0.35, image=None, ep_camera=ep_camera)

                # use IR sensor to find distance to obstacle
                sensor_dist = math.sin(camera_angle) * self.dist
                print("Object distance: ", sensor_dist)
            
            # once lego is identified, pause rotation 
            ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5)
            time.sleep(sleep_time)

            # if the centroid of the bounding box is to the right of camera center, rotate robot clockwise.
            if bb_lego[1][0] > 1280/2:
                ep_chassis.drive_speed(x=0, y=0, z=z_speed/2, timeout=5)
                time.sleep(sleep_time)
                # ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5)
                # time.sleep(sleep_time)

            # if the centroid of the bounding box is to the left of camera center, rotate robot counter-clockwise.
            else:
                ep_chassis.drive_speed(x=0, y=0, z=-z_speed/2, timeout=5)
                time.sleep(sleep_time)
                # ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5)
                # time.sleep(sleep_time)
                
            tol = 100 # pixel tolerance
            # once the center of the lego's bounding box falls within a certain pixel tolerance relative to the camera center, exit the function
            if 1280/2 + tol > bb_lego[1][0] > 1280/2 - tol:
                # if the lego is close, exit the function without calling anything else.
                if bb_lego[1][2] >= h_lego_max:
                    return
                # if the lego is far, return move_robot() (defined below) to move robot towards the target.
                else: return self.move_robot(ep_camera=ep_camera)
    

    # moves the robot towards the target lego as long as the lego is in the camera frame.
    # also moves robot side to side to avoid obstacles during approach.
    def move_robot(self, x_speed = 0.2, y_speed = 0.3, sleep_time = 0.1, h_obstacle_max = 450, h_lego_max = 80, threshold_dist = 300, ep_camera = None):
        print("GIVER: Moving to lego")
        # parameter for camera pointing angle
        camera_angle = math.atan(threshold_dist/100)

        while True:
            # use IR sensor to find distance to obstacle
            sensor_dist = math.sin(camera_angle) * self.dist
            print("Object distance: ", sensor_dist)

            # read in largest (nearest) YOLO bounding box for obstacles and lego
            bb_obstacle = detection.detect_object_in_image(c='obstacle', conf=0.35, image=None, ep_camera=ep_camera)
            bb_lego = detection.detect_object_in_image(c='lego', conf=0.35, image=None, ep_camera=ep_camera)

            # for as long as YOLO sees a lego...
            while bb_lego[0]:

                # if lego is very close, exit function and return look_for_lego() to line robot up more precisely
                if bb_lego[1][2] > h_lego_max:
                    return self.look_for_lego(ep_camera = ep_camera)

                # if obstacles are in view, do the following...
                if bb_obstacle[0] and ((sensor_dist < threshold_dist) or (bb_obstacle[1][2] > h_obstacle_max)):
                    # if an obstacle is in view and its centroid is to the right of the camera center, slide robot left
                    if (bb_obstacle[1][0] > 1280/2):
                        ep_chassis.drive_speed(x=0, y=-y_speed, z=0, timeout=5)
                        time.sleep(10*sleep_time)
                    # if an obstacle is in view and its centroid is to the left of the camera center, slide robot right
                    else:
                        ep_chassis.drive_speed(x=0, y=y_speed, z=0, timeout=5)
                        time.sleep(10*sleep_time)
                
                else:
                    ep_chassis.drive_speed(x=x_speed, y=0, z=0, timeout=5)
                    time.sleep(sleep_time)

                # read in largest (nearest) YOLO bounding box for obstacles and lego
                bb_obstacle = detection.detect_object_in_image(c='obstacle', conf=0.35, image=None, ep_camera=ep_camera)
                bb_lego = detection.detect_object_in_image(c='lego', conf=0.35, image=None, ep_camera=ep_camera)

                # use IR sensor to find distance to obstacle
                sensor_dist = math.sin(camera_angle) * self.dist
                print("Object distance: ", sensor_dist)

            # if the lego block leaves view at any point, call look_for_lego() to re-orient
            return self.look_for_lego(ep_camera=ep_camera)
        
        
    # moves the robot from the lego enclosure to the blue line.
    # also moves robot side to side to avoid obstacles during approach.
    def move_robot2(self, is_left, x_speed = 0.2, y_speed = 0.30, sleep_time = 0.1, h_obstacle_max = 450, threshold_dist = 300, ep_camera = None):
        print("GIVER: Moving to lego")

        # if robot starts off on the left side this is
        # -1 otherwise if it is the right side this is 1
        if is_left:
            sign = 1
        else: 
            sign = -1

        ## STAGE 1 ##
        ep_chassis.drive_speed(x=0, y=0, z=sign*-45, timeout=5)
        time.sleep(20*sleep_time)

        # parameter for camera pointing angle
        camera_angle = math.atan(threshold_dist/100)

        while self.x < 0.4:
            # use IR sensor to find distance to obstacle
            sensor_dist = math.sin(camera_angle) * self.dist
            print("x: ", self.x, "y: ", self.y, "Object: ", sensor_dist)

            # read in largest (nearest) YOLO bounding box for obstacles and lego
            bb_obstacle = detection.detect_object_in_image(c='obstacle', conf=0.35, image=None, ep_camera=ep_camera)

            # if obstacles are in view, do the following...
            if bb_obstacle[0] and ((sensor_dist < threshold_dist) or (bb_obstacle[1][2] > h_obstacle_max)):
                # if an obstacle is in view and its centroid is to the right of the camera center, slide robot left
                if (bb_obstacle[1][0] > 1280/2):
                    ep_chassis.drive_speed(x=0, y=-y_speed, z=0, timeout=5)
                    time.sleep(10*sleep_time)
                # if an obstacle is in view and its centroid is to the left of the camera center, slide robot right
                else:
                    ep_chassis.drive_speed(x=0, y=y_speed, z=0, timeout=5)
                    time.sleep(10*sleep_time)
            
            else:
                ep_chassis.drive_speed(x=x_speed, y=0, z=0, timeout=5)
                time.sleep(sleep_time)
        
        ## STAGE 2 ##
        ep_chassis.drive_speed(x=0, y=0, z=sign*45, timeout=5)
        time.sleep(20*sleep_time)

        # parameter for camera pointing angle
        camera_angle = math.atan(threshold_dist/100)

        while self.x < 1.4:
            # use IR sensor to find distance to obstacle
            sensor_dist = math.sin(camera_angle) * self.dist
            print("x: ", self.x, "y: ", self.y, "Object: ", sensor_dist)

            # read in largest (nearest) YOLO bounding box for obstacles and lego
            bb_obstacle = detection.detect_object_in_image(c='obstacle', conf=0.35, image=None, ep_camera=ep_camera)

            # if obstacles are in view, do the following...
            if bb_obstacle[0] and ((sensor_dist < threshold_dist) or (bb_obstacle[1][2] > h_obstacle_max)):
                # if an obstacle is in view and its centroid is to the right of the camera center, slide robot left
                if (bb_obstacle[1][0] > 1280/2):
                    ep_chassis.drive_speed(x=0, y=-y_speed, z=0, timeout=5)
                    time.sleep(10*sleep_time)
                # if an obstacle is in view and its centroid is to the left of the camera center, slide robot right
                else:
                    ep_chassis.drive_speed(x=0, y=y_speed, z=0, timeout=5)
                    time.sleep(10*sleep_time)
            
            else:
                ep_chassis.drive_speed(x=x_speed, y=0, z=0, timeout=5)
                time.sleep(sleep_time)

        ## STAGE 3 ##
        ep_chassis.drive_speed(x=0, y=0, z=sign*40, timeout=5)
        time.sleep(20*sleep_time)

        while self.x < 1.8:
            # use IR sensor to find distance to obstacle
            sensor_dist = math.sin(camera_angle) * self.dist
            print("x: ", self.x, "y: ", self.y, "Object: ", sensor_dist)

            # read in largest (nearest) YOLO bounding box for obstacles and lego
            bb_obstacle = detection.detect_object_in_image(c='obstacle', conf=0.35, image=None, ep_camera=ep_camera)

            # if obstacles are in view, do the following...
            if bb_obstacle[0] and ((sensor_dist < threshold_dist) or (bb_obstacle[1][2] > h_obstacle_max)):
                # if an obstacle is in view and its centroid is to the right of the camera center, slide robot left
                if (bb_obstacle[1][0] > 1280/2):
                    ep_chassis.drive_speed(x=0, y=-y_speed, z=0, timeout=5)
                    time.sleep(10*sleep_time)
                # if an obstacle is in view and its centroid is to the left of the camera center, slide robot right
                else:
                    ep_chassis.drive_speed(x=0, y=y_speed, z=0, timeout=5)
                    time.sleep(10*sleep_time)
            
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
                    ep_chassis.drive_speed(x=x_speed, y=0, z=0) #x_vel
                    time.sleep(5)
                    ep_chassis.drive_speed(x=0, y=0, z=0)
                    time.sleep(0.5)
                
            else:
                print('Target line is out of view')
                ep_chassis.drive_speed(x=0, y=0, z=-15)
                time.sleep(1)
    

    # make reeciver face endpoint
    def search_endpoint(self, rotational_speed = 10, k = 0.02, ep_camera=None): 
        
        distance = 1000000
        print('RECEIVER: Seaching for Endpoint')

        # want bounding box as close to center of img in horizontal dir
        while np.abs(distance) > 20: # bounding box x-center must be 15 away from center of img

            results = detection.detect_endpoint(ep_camera=ep_camera, show=False)

            if results[0]: # if lego in FOV
                bb = results[1] # bounding box -  array of format [x,y,w,h]
                horizontal_center = bb[0]
                distance = horizontal_center - 1280/2 # finding error in horizontal
                ep_chassis.drive_speed(x=0, y=0, z=k * distance * rotational_speed, timeout=5)

            else:
                ep_chassis.drive_speed(x=0, y=0, z=rotational_speed, timeout=5)

            time.sleep(0.1)

        ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5) # stop rotating
        print('RECEIVER: Facing Endpoint')


    # make receiver go to endpoint
    def move_to_endpoint(self, translation_speed = 0.04, rotational_speed = 10, 
                    k_t = 0.01, k_r = 0.05, ep_camera=None): 
        
        width = 0
        print('RECEIVER: Moving to Endpoint')

        goal_width = 800

        prev_width = 0
        count = 0

        # want bounding box as close to center of img in horizontal dir
        while width < goal_width:

            print(width)
            
            results = detection.detect_endpoint(ep_camera=ep_camera, show=False)

            if results[0]: # if lego in FOV
                
                bb = results[1] # bounding box -  array of format [x,y,w,h] scaled to image size of (384, 640)
                width = bb[2]
                width_error = (goal_width) - width # finding height in box width

                horizontal_center = bb[0]
                horizontal_distance = horizontal_center - 1280/2 - 400 # finding error in horizontal


                if (horizontal_distance > 100):
                    ep_chassis.drive_speed(x=translation_speed * k_t * width_error, y=0,
                                    z= rotational_speed * k_r * horizontal_distance, timeout=5)

                if (horizontal_distance <= 100):
                    ep_chassis.drive_speed(x=translation_speed * k_t * width_error, y=0,
                                    z=0, timeout=5)

                if prev_width >= width:
                    count = count + 1
                else:
                    count = 0
                prev_width = width

                if count >= 3:
                    break

            else:
                ep_chassis.drive_speed(x=0, y=0, z=rotational_speed, timeout=5)

            time.sleep(0.1)

        ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5) # stop moving
        time.sleep(0.1)
        print('RECEIVER: At Endpoint')

    def grab_lego(self, ep_gripper=None, ep_arm=None, x_dist=-30, y_dist=-100, power=50, sleep_time = 0.5):
        print('GIVER: Grabbing')
        ep_arm.move(x=x_dist, y=y_dist).wait_for_completed()
        time.sleep(sleep_time)
        ep_gripper.close(power=power)
        time.sleep(sleep_time)
        ep_arm.move(x=-x_dist, y=-y_dist).wait_for_completed()
        time.sleep(sleep_time)
        print('GIVER: Done grabbing')
        return
    
    def drop_lego(self, ep_gripper=None, ep_arm=None, x_dist=30, y_dist=100, power=50, sleep_time = 0.5):
        print('GIVER: Placing Lego')
        ep_arm.move(x=x_dist, y=y_dist).wait_for_completed()
        time.sleep(sleep_time)
        ep_gripper.open(power=power)
        time.sleep(sleep_time)
        ep_arm.move(x=-x_dist, y=-y_dist).wait_for_completed()
        time.sleep(sleep_time)
        print('GIVER: Done placing Lego')
        return

def sub_data_handler(sub_info):
    distance = sub_info
    distspinner.dist = distance[0]

def sub_position_handler(position_info):
    x, y, z = position_info
    # distspinner.x = y
    # distspinner.y = -x
    # distspinner.z = z

    # change as needed
    distspinner.x = y
    distspinner.y = x
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
        ep_camera.start_video_stream(display=True)

        time.sleep(1)

        for i in range(5):
            # a1) Acquire lego
            distspinner.look_for_lego(ep_camera=ep_camera)
            # a2) Grab lego
            distspinner.grab_lego(ep_gripper=ep_gripper, ep_arm=ep_arm)
            # a3) Orient to target
            distspinner.search_endpoint(x_speed = -0.1, ep_camera=ep_camera)
            # a4) Move to target
            distspinner.move_to_endpoint(ep_camera=ep_camera, is_left=True)
            # a5) Drop lego
            distspinner.drop_lego(ep_gripper=ep_gripper, ep_arm=ep_arm)
            # a6) Rotate back
            ep_chassis.drive_speed(x=0, y=0, z=45, timeout=5)
            time.sleep(4)

        ep_sensor.unsub_distance()
        ep_chassis.unsub_position()
        ep_robot.close()