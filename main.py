import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import maestro
from enum import IntEnum

MOTOR_KEY = 1
TURN_KEY = 2

NARROW_THRESH_SIZE = 80			#threshold for small turns and movements
WIDE_THRESH_SIZE=250			#threshold for when large or faster turns are needed

class RobotDirection(IntEnum):
    RIGHT = 5400			#motor speed for when the turns are small
    HARD_RIGHT = 5400			#motor speed for when the turns are sharp or getting off track too far
    LEFT = 6600
    HARD_LEFT = 6600
    FORWARD = 6000			#motor speed for moving both motors at the same speed

class Robot:
    def __init__(self):
        self.tango = maestro.Controller()
        self.motors = 6000			#starting speed to initialize the motors

        self.moving = False				#initialize the robot to not be moving
        self.direction = RobotDirection.FORWARD		#initialize the robot direction as moving forward

        self.tango.setTarget(MOTOR_KEY, self.motors)		#send the motor speeds to the motor
        self.tango.setTarget(TURN_KEY, int(self.direction))	#send the robot direction to the controller

#method for  propelling the robot forward
    def forward(self):
        self.moving = True
#gradually increase the motor speed so that speed increase is smooth
        for i in range(0, 3):
            self.motors -= 150
            self.tango.setTarget(MOTOR_KEY, self.motors)	#send the motor speed to the motor
            time.sleep(0.2)					#pause before setting the next speed

#method for turning the robot to the right
    def right(self):
        if self.direction != RobotDirection.RIGHT:		#if the robot direction is not already set at turning right, set it to turn right
            self.direction = RobotDirection.RIGHT
            self.tango.setTarget(TURN_KEY, int(self.direction))

#method for turning sharp right or if the robot is getting too far off track
    def hardRight(self):
        if self.direction != RobotDirection.HARD_RIGHT:
            self.direction = RobotDirection.HARD_RIGHT
            self.tango.setTarget(TURN_KEY, int(self.direction))

#method for turning the robot left
    def left(self):
        if self.direction != RobotDirection.LEFT:
            self.direction = RobotDirection.LEFT
            self.tango.setTarget(TURN_KEY, int(self.direction))

#method for turning the robot sharp left or if the robot is getting too far off track
    def hardLeft(self):
        if self.direction != RobotDirection.HARD_LEFT:
            self.direction = RobotDirection.HARD_LEFT
            self.tango.setTarget(TURN_KEY, int(self.direction))

#method for getting the robot to go straight
    def straight(self):
        if self.direction != RobotDirection.FORWARD:
            self.direction = RobotDirection.FORWARD
            self.tango.setTarget(TURN_KEY, int(self.direction))

#method for stopping the robot
    def stop(self):
        self.moving = False
        self.direction = RobotDirection.FORWARD

        self.motors = 6000						#set the motor to neutral
        self.tango.setTarget(TURN_KEY, int(self.direction))
        self.tango.setTarget(MOTOR_KEY, self.motors)

#listener that listens to when the screen is tapped.
#tapping the screen once starts the robot moving. tapping again stops the robot from moving
def mouseCall(evt, x, y, flags, pic):
    global robot
    if evt == cv2.EVENT_LBUTTONDOWN:
        if robot.moving:
            robot.stop()
        else:
            robot.forward()

robot = Robot()

#robot.forward()
#time.sleep(1)
#robot.hardLeft()
#time.sleep(2)
#robot.stop()

def main():
    cv2.namedWindow("Main", cv2.WINDOW_NORMAL)							#draw the main view camera view on the screen
    cv2.setWindowProperty("Main", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN);		#set the screen to full screen

    camera = PiCamera()
    camera.resolution = (640, 480)				#set the resolution of the camera
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size=(640, 480))

    # allow camera to warmup
    time.sleep(0.1)

#continuously read frames from the camera
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        img = frame.array

        cv2.setMouseCallback("Main", mouseCall, img)			#set the mouse listener to listen on the main screen

        height = img.shape[0]
        width = img.shape[1]

        roi_x1 = 0						#defines the left side of the region of interest to start at far left side of the screen
        roi_y1 = int(height/2)					#sets the top of the region of interest to be the middle of the screen
        roi_x2 = width						#sets the right side of the region of interest to end at the far right of the screen
        roi_y2 = height						#sets the bottom of the region of interest to be the bottom of the screen
        img_roi = img[roi_y1:roi_y2, roi_x1:roi_x2]				#create an image that covers the ROI
        cv2.rectangle(img, (roi_x1, roi_y1), (roi_x2, roi_y2), (0, 255, 255), 2)		#draw a rectangle on around the ROI
        hsv_img = cv2.cvtColor(img_roi, cv2.COLOR_BGR2HSV)					#convert BGR to HSV on the ROI

        minHSV = np.array([0, 30, 209])				#defines the minimum value range of the yellow "line"
        maxHSV = np.array([57, 193, 255])			#defines the maximim value range of the yellow "line"

        #minHSV = np.array([14, 149, 237])
        #maxHSV = np.array([24, 193, 255])

        color_detected = cv2.inRange(hsv_img, minHSV, maxHSV)		#thresholds the image to show only the color within the range defined above

        contours, _ = cv2.findContours(color_detected, 1, cv2.CHAIN_APPROX_NONE)		#detect the shapes in the thresholded image

#if the contour size is larger than 0
        if len(contours) > 0:
            max_contour = max(contours, key=cv2.contourArea)		#find the largest contour
            max_contour_COG = cv2.moments(max_contour)			#find the center of the largest contour

            cv2.drawContours(img_roi, contours, -1, (0, 255, 0), 2)	#draw all the contours on the ROI image

#if there is a center to the contour and not dividing by zero
            if max_contour_COG != None and max_contour_COG['m00'] != 0:
                cx = int(max_contour_COG['m10'] / max_contour_COG['m00'])		#find the x value of the center of the contour
                cy = int(max_contour_COG['m01'] / max_contour_COG['m00'])		#find the y value of the center of the contour

                cv2.line(img_roi, (cx, 0), (cx, height), (255, 0, 0), 2)	#draw a horizontal line across the screen that goes through the center of the contour
                cv2.line(img_roi, (0, cy), (width, cy), (255, 0, 0), 2)		#draw a vertical line down the screen that goes through the center of the contour

                narrowLeftBound = int((width/2)-NARROW_THRESH_SIZE/2)		#define the left boundary for where the contour the robot is following is located
                narrowRightBound = int((width/2)+NARROW_THRESH_SIZE/2)		#define the right boundary for where the contour the robot is following is located
#draw a rectangle around the boundaries that the contour should stay in
                cv2.rectangle(img_roi, (narrowLeftBound, -5), (narrowRightBound, height+5), (0,0,255), 2)

#define a boundary that is farther out so that the motor speed can be increased to bring the robot back into line faster
                wideLeftBound = int((width/2)-WIDE_THRESH_SIZE/2)
                wideRightBound = int((width/2)+WIDE_THRESH_SIZE/2)
                cv2.rectangle(img_roi, (wideLeftBound, -5), (wideRightBound, height+5), (255,0,255), 2)	#draw a rectangle around the outer boundary

#if the center of the contour is in the narrow bound turn slowly to keep the center in the view
#if the center of the contour is in the outer bound turn more quickly to keep the center of the contour in the view
                if robot.moving:
                    if cx >= narrowRightBound:
                        robot.right()
                    elif cx >= wideRightBound:
                        robot.hardRight()
                    elif cx < narrowRightBound and cx > narrowLeftBound:
                        robot.straight()
                    elif cx <= narrowLeftBound:
                        robot.left()
                    elif cx <= wideLeftBound:
                        robot.hardLeft()

#if no contours are detected show a red circle in the bottom right hand contour and stop the robot
        else:
            cv2.circle(img, (width - 50, height - 50), 40, (0,0,255), -1)
            robot.stop()
#if the robot is not moving draw a green circle in the bottom left hand corner
        if not robot.moving:
            cv2.circle(img, (50, height - 50), 40, (0,255,0), -1)

        cv2.imshow("Main", img)		#show the screen with the circles drawn on it

        key = cv2.waitKey(1) & 0xFF	#listen for the screen to be touched to know when to stop the program

        rawCapture.truncate(0)

        if key == ord("q"):		#stop the program from running
            break

    cv2.destroyAllWindows()


main()
