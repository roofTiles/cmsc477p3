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
    def look_for_lego(self, is_Left = True, x_speed = 0.5, z_speed = 10, sleep_time = 0.1, w_obstacle_max = 450, h_lego_max = 90, threshold_dist = 300, ep_camera = None):
        print("GIVER: Looking for lego")
        # parameter for camera pointing angle
        camera_angle = math.atan(threshold_dist/100) 

        # if robot starts off on the left side this is
        # -1 otherwise if it is the right side this is 1
        if is_left:
            sign = 1
        else: 
            sign = -1

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
                if (rotation < 360) or (bb_obstacle[1][3] > w_obstacle_max or sensor_dist < threshold_dist):
                    ep_chassis.drive_speed(x=0, y=0, z=sign*-z_speed, timeout=5)
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
                
            tol = 200 # pixel tolerance
            # once the center of the lego's bounding box falls within a certain pixel tolerance relative to the camera center, exit the function
            if 1280/2 + tol > bb_lego[1][0] > 1280/2 - tol:
                # if the lego is close, exit the function without calling anything else.
                if bb_lego[1][3] >= h_lego_max:
                    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5)
                    time.sleep(sleep_time)
                    return [distspinner.x, distspinner.y, distspinner.z]
                # if the lego is far, return move_to_lego() (defined below) to move robot towards the target.
                else: return self.move_to_lego(ep_camera=ep_camera)
    
    # moves the robot towards the target lego as long as the lego is in the camera frame.
    # also moves robot side to side to avoid obstacles during approach.
    def move_to_lego(self, x_speed = 0.2, y_speed = 0.15, sleep_time = 0.1, w_obstacle_max = 450, h_lego_max = 90, threshold_dist = 300, ep_camera = None):
        print("GIVER: Moving to lego")
        # parameter for camera pointing angle
        camera_angle = math.atan(threshold_dist/100)

        while True:
            # use IR sensor to find distance to obstacle
            sensor_dist = math.sin(camera_angle) * self.dist
            print("Object distance: ", sensor_dist)
            print("x: ", self.x, "y: ", self.y, "z: ", self.z, "Object: ", sensor_dist)

            # read in largest (nearest) YOLO bounding box for obstacles and lego
            bb_obstacle = detection.detect_object_in_image(c='obstacle', conf=0.35, image=None, ep_camera=ep_camera)
            bb_lego = detection.detect_object_in_image(c='lego', conf=0.35, image=None, ep_camera=ep_camera)

            # for as long as YOLO sees a lego...
            while bb_lego[0]:

                # if lego is very close, exit function and return look_for_lego() to line robot up more precisely
                if bb_lego[1][3] > h_lego_max:
                    return [distspinner.x, distspinner.y, distspinner.z] # self.look_for_lego(ep_camera = ep_camera)

                # if obstacles are in view, do the following...
                if bb_obstacle[0] and ((sensor_dist < threshold_dist) or bb_obstacle[1][2] > w_obstacle_max): 
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
        
        
    def move_robot(self, moving_x = True, distance = 0, diff = 0, x_speed = 0.2, y_speed = 0.1, sleep_time = 0.1, w_obstacle_max = 450, threshold_dist = 300, ep_camera = None):
        print("GIVER: Moving to lego")

        # parameter for camera pointing angle
        camera_angle = math.atan(threshold_dist/100)

        if moving_x:
                param = self.x

        else:
                param = self.y

        while abs(param - diff) < distance:
            # print("Distance left: ",)
            if moving_x:
                param = self.x

            else:
                param = self.y

            # use IR sensor to find distance to obstacle
            sensor_dist = math.sin(camera_angle) * self.dist
            print("x: ", self.x, "y: ", self.y, "z: ", self.z, "Object: ", sensor_dist)

            # read in largest (nearest) YOLO bounding box for obstacles and lego
            bb_obstacle = detection.detect_object_in_image(c='obstacle', conf=0.35, image=None, ep_camera=ep_camera)

            # if obstacles are in view, do the following...
            if bb_obstacle[0] and ((sensor_dist < threshold_dist or bb_obstacle[1][2] > w_obstacle_max)):
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

        ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5)
        time.sleep(sleep_time)
        print("done moving")

        return [distspinner.x, distspinner.y, distspinner.z]
    

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

    def grab_lego(self, ep_gripper=None, ep_arm=None, x_dist=10, y_dist=-120, power=100, sleep_time = 1):
        print('GIVER: Grabbing')
        ep_arm.move(x=x_dist, y=y_dist).wait_for_completed()
        ep_gripper.close(power=power)
        time.sleep(sleep_time)
        ep_arm.move(x=-x_dist, y=-y_dist).wait_for_completed()
        print('GIVER: Done grabbing')
        return
    
    def drop_lego(self, ep_gripper=None, ep_arm=None, x_dist=10, y_dist=-120, power=100, sleep_time = 1):
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
    distspinner.x = x
    distspinner.y = y

def sub_attitude_info_handler(attitude_info):
    yaw, pitch, roll = attitude_info
    distspinner.z = yaw
    print("chassis attitude: yaw:{0}, pitch:{1}, roll:{2} ".format(yaw, pitch, roll))


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
    ep_chassis.sub_attitude(freq=10, callback=sub_attitude_info_handler)

    ep_camera.start_video_stream(display=True)

    time.sleep(1)
    ep_gripper.open(power=100)
    time.sleep(1)
    ep_arm.move(x=100, y=-10).wait_for_completed()

    is_left = False
    if is_left:
        sign = 1
    else: 
        sign = -1

    # a1) Move giver to center of workspace
    encoder_vals1 = distspinner.move_robot(moving_x = True, distance = 1, diff = 0, ep_camera=ep_camera)  
    
    # a2) Look for and move to lego
    encoder_vals2 = distspinner.look_for_lego(ep_camera=ep_camera, is_Left=False)
    
    # a3) Grab lego
    distspinner.grab_lego(ep_gripper=ep_gripper, ep_arm=ep_arm)
    
    # a4) Rotate back towards starting point
    rotation = abs(int(abs(encoder_vals2[2]-encoder_vals1[2]))%360 - 180)  
    # if encoder_vals2[1] < 0:
    #     rotation = -rotation
    # print(rotation)
    ep_chassis.drive_speed(x=0, y=0, z=rotation-10, timeout=5)
    time.sleep(1)

    # a5) Move horizontally to back right corner
    encoder_vals3 = distspinner.move_robot(moving_x = True, distance = 1, diff = encoder_vals2[0], ep_camera=ep_camera)  
    ep_chassis.drive_speed(x=0, y=0, z=-90, timeout=5)
    time.sleep(1)

    # a6) Move vertically toward blue 
    encoder_vals4 = distspinner.move_robot(moving_x = False, distance = 1.5, diff = encoder_vals3[1], ep_camera=ep_camera)
    ep_chassis.drive_speed(x=0, y=0, z=-80, timeout=5)
    time.sleep(1)

    # a7) Move horizontally to blue line
    encoder_vals3 = distspinner.move_robot(moving_x = True, distance = 1, diff = encoder_vals4[0], ep_camera=ep_camera)  

    # a8) Find and move vertically towards blue line
    distspinner.move_to_line(x_speed = 0.1, ep_camera=ep_camera)
    
    # a9) Drop lego
    distspinner.drop_lego(ep_gripper=ep_gripper, ep_arm=ep_arm)

    for i in range(5):
        # b1) Acquire lego
        distspinner.look_for_lego(ep_camera=ep_camera)
        # b2) Grab lego
        distspinner.grab_lego(ep_gripper=ep_gripper, ep_arm=ep_arm)
        # b3) Move back to line
        distspinner.move_to_line(x_speed = 0.3, ep_camera=ep_camera)
        # b4) Drop lego
        distspinner.drop_lego(ep_gripper=ep_gripper, ep_arm=ep_arm)

    ep_sensor.unsub_distance()
    ep_chassis.unsub_position()
    ep_robot.close()
