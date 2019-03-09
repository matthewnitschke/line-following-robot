import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import maestro

MOTOR_KEY = 1
TURN_KEY = 2

CENTER_THRESH_SIZE = 80

class Robot:
    def __init__(self):
        self.tango = maestro.Controller()
        self.motors = 6000
        self.turn = 6000

        self.movingForward = False
        self.movingRight = False
        self.movingLeft = False

    def forward(self):
        self.movingForward = True
        for i in range(0, 3):
            self.motors -= 140
            self.tango.setTarget(MOTOR_KEY, self.motors)
            time.sleep(0.2)

    def right(self):
        self.movingRight = True
        self.turn -= 400
        self.tango.setTarget(TURN_KEY, self.turn)

    def left(self):
        self.movingLeft = True
        self.turn += 400
        self.tango.setTarget(TURN_KEY, self.turn)

    def straight(self):
        self.movingLeft = False
        self.movingRight = False
        self.turn = 6000
        self.tango.setTarget(TURN_KEY, self.turn)

    def stop(self):
        self.movingForward = False
        self.movingLeft = False
        self.movingRight = False
        self.turn = 6000
        self.motors = 6000
        self.tango.setTarget(TURN_KEY, self.turn)
        self.tango.setTarget(MOTOR_KEY, self.motors)


def mouseCall(evt, x, y, flags, pic):
    global robot
    if evt == cv2.EVENT_LBUTTONDOWN:
        if robot.movingForward:
            robot.stop()
        else:
            robot.forward()

robot = Robot()

cv2.namedWindow("Main", cv2.WINDOW_NORMAL)
cv2.setWindowProperty("Main", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN);

camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))

# allow camera to warmup
time.sleep(0.1)

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    img = frame.array

    cv2.setMouseCallback("Main", mouseCall, img)

    height = img.shape[0]
    width = img.shape[1]

    roi_x1 = 0
    roi_y1 = int(height/2)
    roi_x2 = width
    roi_y2 = height
    img_roi = img[roi_y1:roi_y2, roi_x1:roi_x2]
    cv2.rectangle(img, (roi_x1, roi_y1), (roi_x2, roi_y2), (0, 255, 255), 2)

    hsv_img = cv2.cvtColor(img_roi, cv2.COLOR_BGR2HSV)

    minHSV = np.array([0, 30, 209])
    maxHSV = np.array([57, 193, 255])

    #minHSV = np.array([14, 149, 237])
    #maxHSV = np.array([24, 193, 255])

    color_detected = cv2.inRange(hsv_img, minHSV, maxHSV)

    contours, _ = cv2.findContours(color_detected, 1, cv2.CHAIN_APPROX_NONE)
    if len(contours) > 0:
        max_contour = max(contours, key=cv2.contourArea)
        max_contour_COG = cv2.moments(max_contour)

        cv2.drawContours(img_roi, contours, -1, (0, 255, 0), 2)

        if max_contour_COG != None and max_contour_COG['m00'] != 0:
            cx = int(max_contour_COG['m10'] / max_contour_COG['m00'])
            cy = int(max_contour_COG['m01'] / max_contour_COG['m00'])

            cv2.line(img_roi, (cx, 0), (cx, height), (255, 0, 0), 2)
            cv2.line(img_roi, (0, cy), (width, cy), (255, 0, 0), 2)

            leftBound = int((width/2)-CENTER_THRESH_SIZE/2)
            rightBound = int((width/2)+CENTER_THRESH_SIZE/2)
            cv2.rectangle(img_roi, (leftBound, -5), (rightBound, height+5), (0,0,255), 2)

            if robot.movingForward:
                if cx >= rightBound and not robot.movingRight:
                    robot.right()
                if cx < rightBound and cx > leftBound:
                    robot.straight()
                if cx <= leftBound and not robot.movingLeft:
                    robot.left()
    else:
        cv2.circle(img, (width - 50, height - 50), 40, (0,0,255), -1)

    if not robot.movingForward:
        cv2.circle(img, (50, height - 50), 40, (0,255,0), -1)

    cv2.imshow("Main", img)

    key = cv2.waitKey(1) & 0xFF

    rawCapture.truncate(0)

    if key == ord("q"):
        break

cv2.destroyAllWindows()
