from ultralytics import YOLO
from robomaster import robot
from robomaster import camera
import cv2
import numpy as np

model = YOLO("models/best.pt")

# c is the desired class type for obj detection with the following
# options 'robot' and 'lego'

# conf is the desired confidence interval for detection
# image to be used for obj detection. If none given then
# it will use whatever is currently shown on the image.

# returns the bounding box array of format [x,y,w,h]
# normalized to original image size
def detect_object_in_image(c='robot', conf=0.5, image=None, ep_camera=None):

    if image == None:
        image = ep_camera.read_cv2_image(strategy="newest", timeout=5)


    classes = []

    if c == 'both':
        classes = [0,1]

    if c == 'lego':
        classes=[0]
    if c == 'robot':
        classes = [1]
        
    results = model.predict(source=image, show=False, conf = conf,
                            imgsz=(384, 640), classes=classes)
    
    if len(results) > 1:
        raise Exception("More than one lego detected!")

    if len(results) == 1: # only one detection found
        result = results[0].cpu().numpy()
        boxes = result.boxes.xywhn

        if len(boxes) == 0: #nothing found
            return [False, None]

        bb = boxes[0]

        # scales to input pixel coords
        bb[0] = bb[0]*640
        bb[1] = bb[1]*384
        bb[2] = bb[2]*640
        bb[3] = bb[3]*384
        return [True, boxes[0]]

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
    img = cv2.imread("C:\\umd\\senior\\cmsc477\\Project 2\\proj_code\\endpoint_test\\6.jpg")
    img = cv2.resize(img, (640, 384))
    detect_endpoint(image=img)
            
            
    
    
