from ultralytics import YOLO
from robomaster import robot
from robomaster import camera
import cv2
import numpy as np
import time

# model = YOLO("models/proj3.pt")
model = YOLO("models/betterlego.pt")

# c is the desired class type for obj detection with the following
# options 'obstacle' and 'robot'

# conf is the desired confidence interval for detection
# image to be used for obj detection. If none given then
# it will use whatever is currently shown on the image.

# returns the bounding box array of format [x,y,w,h]
# normalized to original image size
def detect_object_in_image(c='obstacle', conf=0.5, image=None, ep_camera=None):

    if image == None:
        image = ep_camera.read_cv2_image(strategy="newest", timeout=5)

    print(image.shape)

    classes = []

    if c == 'all':
        classes = [0,1,2,3]
    if c == 'obstacle':
        classes = [0,2,3]
    if c == 'cone':
        classes = [0]
    if c == 'lego':
        classes = [1]
    if c == 'box':
        classes=[2]
    if c == 'robot':
        classes = [3]
        
    results = model.predict(source=image, show=False, conf = 0.4,
                            imgsz=(384, 640), classes=classes)

    if len(results) == 1: # no detections
        boxes = results[0].cpu().numpy().boxes.xywhn
        if len(boxes) == 0:
            return [False, None]

    tallest_centroid_ind = 0
    tallest_centroid = 0
    for i, r in enumerate(results):
        box = (r.cpu().numpy().boxes.xywhn)[0]
        height = box[1]
        width = box[2]
        if height > tallest_centroid:
            tallest_centroid = height
            tallest_centroid_ind = i

    closest_obstacle = results[tallest_centroid_ind]

    bb = (closest_obstacle.cpu().numpy().boxes.xywhn)[0]

    # scales to input pixel coords
    bb[0] = bb[0]*1280
    # print(bb[0])
    # if np.abs(bb[0]-(1280/2)) >= 1280/3:
    #     return [False, None]
    
    bb[1] = bb[1]*720
    bb[2] = bb[2]*1280
    bb[3] = bb[3]*720

    corners = []
    corners.append([bb[0] - bb[2]/2, bb[1] - bb[3]/2])
    corners.append([bb[0] - bb[2]/2, bb[1] + bb[3]/2])
    corners.append([bb[0] + bb[2]/2, bb[1] + bb[3]/2])
    corners.append([bb[0] + bb[2]/2, bb[1] - bb[3]/2])
    corners = np.asarray(corners)

    pts = corners.reshape((-1, 1, 2)).astype(np.int32)
    image = cv2.polylines(image, [pts], isClosed=True, color=(0, 0, 255), thickness=5)
    # cv2.circle(image, (bb[0], bb[1]), 5, (0, 0, 255), -1)
    cv2.imshow("img", image)
    cv2.waitKey(10)
    
    return [True, bb]

# this function returns the bounding box of the endpoint
# in the image. color parameter indicates what color
# endpoint to look for (yellow or red)
# returns [x,y,w,h]
def detect_endpoint(image=None, color="yellow", show=True, ep_camera=None):

    if image == None:
        image = ep_camera.read_cv2_image(strategy="newest", timeout=5)

    if show:
        original = image

    # masking all other colors except yellow
    if color=="yellow":
        mask = cv2.inRange(image, np.asarray([0, 200, 200]), np.asarray([150, 255, 255])) #bgr channel order
        image = cv2.bitwise_and(image, image, mask = mask)

    # performing erosion to erase false positives
    kernel = np.ones((2, 2), np.uint8)
    image = cv2.erode(image, kernel, iterations=5)

    # convert image to gray scale
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # applying otsu binarization
    thres, image = cv2.threshold(image, 0, 255,
                          cv2.THRESH_OTSU)
    
    # get contours of image
    cnts, hierarchy = cv2.findContours(image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    cnt = sorted(cnts, key=cv2.contourArea, reverse=True) # sort by area

    if len(cnt) == 0: # no endpoint found
        return [False, None]
    
    endpoint_cnt = cnt[0]
    
    # get bb around endpoint contour
    x,y,w,h = cv2.boundingRect(endpoint_cnt)

    if w < 25:
        return [False, None]

    # show where endpoint being detected
    if show:
        original = cv2.rectangle(original,(x,y),(x+w,y+h),(0,255,0),2)
        cv2.imshow('Location of endpoint', original)
        cv2.waitKey(0)

    return [True, [x, y, w, h]]


if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_chassis = ep_robot.chassis
    ep_camera = ep_robot.camera
    ep_gripper = ep_robot.gripper
    ep_arm = ep_robot.robotic_arm

    ep_camera.start_video_stream(display=True)

    #ep_arm.move(x=100, y=-50).wait_for_completed()

    #while True:
        #print(detect_object_in_image(c='box', ep_camera=ep_camera))
    
            
            
    
    
