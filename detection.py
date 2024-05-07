from ultralytics import YOLO
from robomaster import robot
from robomaster import camera
import cv2
import numpy as np

model = YOLO("models/proj3.pt")

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

    if c == 'both':
        classes = [0,1]
    if c == 'robot':
        classes=[0]
    if c == 'obstacle':
        classes = [1]
        
    results = model.predict(source=image, show=False, conf = conf,
                            imgsz=(384, 640), classes=classes)

    if len(results) == 1: # no detections
        boxes = results[0].cpu().numpy().boxes.xywhn
        if len(boxes) == 0:
            return [False, None]

    biggest_height_index = 0
    biggest_height = 0
    for i, r in enumerate(results):
        box = (r.cpu().numpy().boxes.xywhn)[0]
        height = box[3]
        if height > biggest_height:
            biggest_height = height
            biggest_height_index = i

    closest_obstacle = results[biggest_height_index]
    bb = (closest_obstacle.cpu().numpy().boxes.xywhn)[0]
    
    # scales to input pixel coords
    bb[0] = bb[0]*1280
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
    cv2.imshow("img", image)
    cv2.waitKey(10)
    
    #            cv2.circle(img, tuple(res.center.astype(np.int32)), 5, (0, 0, 255), -1)
    
    return [True, bb]


if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_chassis = ep_robot.chassis
    ep_camera = ep_robot.camera
    ep_gripper = ep_robot.gripper
    ep_arm = ep_robot.robotic_arm

    #ep_camera.start_video_stream(display=True)

    while True:
        print(detect_object_in_image(c='both', ep_camera=ep_camera))
    
            
            
    
    
