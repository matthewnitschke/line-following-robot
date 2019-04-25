import time
import maestro
import threading
import cv2
from picamera import PiCamera
from picamera.array import PiRGBArray
import numpy as np
from client import ClientSocket

pink_range = [[151, 77, 193], [177, 187, 248]]
cone_range = [[133, 124, 124], [255, 255, 255]]
#green_goal_range = [[35, 93, 105], [59, 192, 201]] #up close of green poster board
green_goal_range = [[29, 101, 131], [162, 255, 220]] #green goal as viewed from starting area
mining_marker_range = [[94, 107, 129], [255, 255, 255]] # pink board marker color range

contour_max_size = 2700	# max sixe that contour can be when searching for a end zone marker


#yellow line range
mining_line_range = [[5, 48, 190], [25, 74, 255]] # [[0, 110, 134], [101, 255, 239]] #<-expanded original-> #[[0, 22, 211], [81, 88, 255]] 	#yellow line

class Robot:
    MOTORS = 1
    TURN = 2
    BODY = 0
    HEADTILT = 4
    HEADTURN = 3

    # OLD ROBOT
    SHOULDER = 5
    MID_ARM_X = 6
    MID_ARM_Y = 7
    LOW_ARM_Y = 8
    HAND_ROTATE = 9
    HAND = 10

    TURN_INCREMENTOR = 1300
    MOTOR_INCREMENTOR = 1000 # 500

    # HEAD LOCATIONS
    HEAD_FACE_DETECT = 6700
    HEAD_FORWARD = 6400
    HEAD_ICE_DETECT = 5700
    HEAD_LOWEST = 4500

    def __init__(self):
        self.tango = maestro.Controller()
        self.setup_targets()
        IP = '10.200.39.59'
        PORT = 5010
        self.client = ClientSocket(IP, PORT)

    def setup_targets(self):
        targets = [
            Robot.MOTORS,
            Robot.TURN,
            Robot.HEADTILT,
            Robot.HEADTURN,
            Robot.SHOULDER,
            Robot.MID_ARM_X,
            Robot.MID_ARM_Y,
            Robot.LOW_ARM_Y,
            Robot.HAND_ROTATE,
            Robot.HAND,
            Robot.BODY,
        ]

        # set all targets to 6000 (zero out)
        for target in targets:
            self.tango.setTarget(target, 6000)

        #self.tango.setTarget(Robot.HEADTILT, 4000)
        self.tango.setTarget(Robot.HEADTILT, Robot.HEAD_FORWARD)
        #self.tango.setTarget(Robot.HEADTILT, 6000)

    # method to move the head up and down
    def tilt_head(self, target):
        self.tango.setTarget(Robot.HEADTILT, target)

	# moves the arm in front of the face to be able to detect the color
    def arm_to_face(self):
        self.tango.setTarget(Robot.SHOULDER, 4000)
        time.sleep(.2)
        self.tango.setTarget(Robot.MID_ARM_X, 8500)		
        self.tango.setTarget(Robot.MID_ARM_Y, 6700)
        self.tango.setTarget(Robot.LOW_ARM_Y, 8500)
        self.tango.setTarget(Robot.HAND_ROTATE, 7000) # rotate the hand so the marker is in front of the camera
        self.open_hand()
        time.sleep(1)
        self.open_hand()
    
    # move the arm out to the drop position
    def arm_to_drop(self):
        self.tango.setTarget(Robot.MID_ARM_X, 7000)
        self.tango.setTarget(Robot.MID_ARM_Y, 6000)
        self.tango.setTarget(Robot.LOW_ARM_Y, 6000)
        time.sleep(.2)
        self.tango.setTarget(Robot.SHOULDER, 4800)
    
    def arm_to_hide(self):
        self.tango.setTarget(Robot.MID_ARM_X, 4000)
        self.tango.setTarget(Robot.MID_ARM_Y, 6000)
        self.tango.setTarget(Robot.LOW_ARM_Y, 6000)
        time.sleep(.2)
        self.tango.setTarget(Robot.SHOULDER, 6000)

	#close the hand
    def close_hand(self):
        self.tango.setTarget(Robot.HAND, 6700)
    
    #open the hand
    def open_hand(self):
        self.tango.setTarget(Robot.HAND, 5000)

	#turn robot to the right. Robot turns in small increments rather than trying to move smoothly
    def step_right(self):
        self.tango.setTarget(Robot.TURN, 6000 - Robot.TURN_INCREMENTOR)
        time.sleep(.2)
        self.tango.setTarget(Robot.TURN, 6000)

	#turn robot to the left. Robot turns in small increments rather than trying to move smoothly
    def step_left(self):
        self.tango.setTarget(Robot.TURN, 6000 + Robot.TURN_INCREMENTOR)
        time.sleep(.2)
        self.tango.setTarget(Robot.TURN, 6000)

	#move robot forward
    def step_forward(self):
        self.tango.setTarget(Robot.MOTORS, 6000 - Robot.MOTOR_INCREMENTOR)
        time.sleep(.5)
        self.tango.setTarget(Robot.MOTORS, 6000)

    #move robot backward
    def step_backward(self):
        self.tango.setTarget(Robot.MOTORS, 6000 + Robot.MOTOR_INCREMENTOR)
        time.sleep(.2)
        self.tango.setTarget(Robot.MOTORS, 6000)

	#stop the robot
    def stop(self):
        self.tango.setTarget(Robot.MOTORS, 6000)

    def say(self, text):
        self.client.sendData(text)
        print("Robot Says: " + str(text))

class RobotActions:
    def __init__(self, robot, vision):
        self.robot = robot
        self.vision = vision

	#robot spins to the left until it detects the object that is passed in
    def _rotate_until_detect(self, found_var):
         while True:
            if found_var():
                return
            robot.step_left()
            time.sleep(.5)

	#wait for the pink ice to be presented then accept it
    def accept_ice(self):
        self.vision.detect_ice()
        self.robot.tilt_head(Robot.HEAD_ICE_DETECT)
        self.robot.arm_to_face()

        #wait for the ice to be put in the hand. Then grab it and drop the arm
        while True:
            if self.vision.pink_ice_in_hand:
                self.robot.close_hand()
                time.sleep(2)
                self.robot.arm_to_hide()
                break

            time.sleep(.3)
#            robot.say("still looking for that pink ice")
#            time.sleep(5)
        robot.say("that's the good stuff")

	#drive to the human
    def goto_human(self):
        # Look for human
        self.robot.tilt_head(Robot.HEAD_FACE_DETECT)
        self.vision.detect_face()

        self._rotate_until_detect(lambda: self.vision.face_in_thresh)			#turn until the face is within the threshold

        # Drive to human. Threshold for the size of the face
        maxFaceArea = 25000
        minFaceArea = 10000

        robot.say("human detected")
        time.sleep(.5)
        while True:

            if self.vision.face_in_thresh:
                (_, _, face_w, face_h) = self.vision.face_bounds
                faceArea = face_w * face_h

                if (faceArea > maxFaceArea):				#if the face is too close, move back
                    self.robot.step_backward()
                elif (faceArea < minFaceArea):				#if the face is too far, move closer
                    self.robot.step_forward()
                else:
                    break
            else:
                face_x = self.vision.face_bounds[0]
                center_x = int(self.vision.width/2)
                if (face_x > center_x):
                    self.robot.step_right()
                elif (face_x < center_x):
                    self.robot.step_left()

            time.sleep(.3)
        robot.say("Requesting the pink ice")

    def drop_ice(self):
        self.robot.arm_to_drop()
        time.sleep(2)
        self.robot.open_hand()

    def goto_boundary_line(self, color_range):
        self.robot.tilt_head(Robot.HEAD_FORWARD)
        self.vision.detect_boundary_line(color_range)
        self._rotate_until_detect(lambda: self.vision.boundary_endpoint_in_thresh)		#look for the mining zone flag
        head_tilt = 6000 # move the head up or down to neutral
        head_tilt_increment = 200

        while True:
            if self.vision.boundary_line_visible: # move forward while robot can see the line for the mining zone
                self.robot.step_forward()
            elif head_tilt > 4000: # if the robot loses the line, tilt the head down to and continue moving forward
                head_tilt -= head_tilt_increment
                self.robot.tilt_head(head_tilt)
            else: # after the head has been tilted down to see the line and the robot can't see the line we have reached the line
                break
            time.sleep(.3)

        for i in range(0, 2):
            self.robot.step_forward()
            time.sleep(.5)

	#line up for the pink goal
    def goto_goal(self):
        self.robot.tilt_head(Robot.HEAD_FORWARD)
        self.vision.detect_goal()
        self._rotate_until_detect(lambda: self.vision.goal_in_thresh)

        head_tilt = Robot.HEAD_FORWARD
        head_tilt_increment = 100
        while True:
            if head_tilt <= Robot.HEAD_LOWEST:
                break
            elif self.vision.goal_in_thresh:
                self.robot.step_forward()
            elif self.vision.goal_visible:
                goal_x = self.vision.goal_cords[0]
                center_x = int(self.vision.width/2)
                if (goal_x > center_x):
                    self.robot.step_right()
                elif (goal_x < center_x):
                    self.robot.step_left()
            elif head_tilt > Robot.HEAD_LOWEST:
                head_tilt -= head_tilt_increment
                self.robot.tilt_head(head_tilt)
            
            time.sleep(.3)


def mouseCall(evt, x, y, flags, pic):
    if evt == cv2.EVENT_LBUTTONDOWN:
        print(x, y, pic[y][x])

class Vision(threading.Thread):
    def __init__(self):
        super(Vision, self).__init__()
        self.width = 640
        self.height = 480
        self.face_cascade = cv2.CascadeClassifier('data/haarcascades/haarcascade_frontalface_default.xml')

        self.current_action = None

        self.pink_ice_in_hand = False
        self.face_in_thresh = False
        self.face_bounds = None

        self.boundary_endpoint_color_range = None
        self.boundary_endpoint_in_thresh = False
        self.boundary_line_visible = False

        self.goal_visible = False
        self.goal_cords = None
        self.goal_in_thresh = False
        self.goal_in_y_thresh = False

	#put robot in detect-face mode
    def detect_face(self):
        self.face_in_thresh = False
        self.face_bounds = None
        self.current_action = "detect-face"

       #put robot in detect-ice mode
    def detect_ice(self):
        self.pink_ice_in_hand = False
        self.current_action = "detect-ice"

	#put robot in detect endopoint mode
    def detect_boundary_line(self, boundary_endpoint_color_range):
        self.boundary_endpoint_in_thresh = False
        self.boundary_line_visible = False
        self.boundary_endpoint_color_range = boundary_endpoint_color_range
        self.current_action = "detect-boundary-line"

    def detect_goal(self):
        self.goal_visible = False
        self.goal_in_thresh = False
        self.goal_in_y_thresh = False
        self.current_action = "detect-goal"

	#detect a color in the value range passed in
    def _color_detected(self, src, color_range, min_area):
        in_range = cv2.inRange(src, np.array(color_range[0]), np.array(color_range[1]))
        in_range = cv2.dilate(in_range, None, iterations=5)
        in_range = cv2.erode(in_range, None, iterations=1)
        h, s, v, _ = np.uint8(cv2.mean(in_range))
        avg_sum = h + s + v

        return avg_sum > min_area

    def _run_detect_goal(self, img, hsv_img, left_threshold, right_threshold):
        cv2.rectangle(img, (left_threshold, -2), (right_threshold, 481), (0,255,255), 2)

        y_thresh_height = 100
        top_threshold = self.height - y_thresh_height
        bottom_threshold = self.height
        cv2.rectangle(img, (-2, top_threshold), (self.width+2, bottom_threshold), (255,0,255), 2)


        # in_range = cv2.inRange(hsv_img, np.array(mining_marker_range[0]), np.array(mining_marker_range[1]))
        in_range = cv2.inRange(hsv_img, np.array(cone_range[0]), np.array(cone_range[1]))
        in_range = cv2.dilate(in_range, None, iterations=5)
        in_range = cv2.erode(in_range, None, iterations=1)

        contours, _ = cv2.findContours(in_range, 1, cv2.CHAIN_APPROX_NONE)
        if len(contours) > 0:
            max_contour = max(contours, key=cv2.contourArea)		#find the largest contour
            max_contour_COG = cv2.moments(max_contour)			#find the center of the largest contour
            _, _, w, h = cv2.boundingRect(max_contour)
            contourArea = w*h
            if contourArea >= contour_max_size:					#only look at contours greater than 7500
                cv2.drawContours(img, max_contour, -1, (0, 255, 0), 2)	#draw all the contours on the ROI image

                if max_contour_COG != None and max_contour_COG['m00'] != 0:
                    cx = int(max_contour_COG['m10'] / max_contour_COG['m00']) # find the x value of the center of the contour
                    cy = int(max_contour_COG['m01'] / max_contour_COG['m00']) # find the y value of the center of the contour
                    
                    self.goal_visible = True
                    self.goal_cords = [cx, cy]

                    self.goal_in_thresh = cx >= left_threshold and cx <= right_threshold
                    self.goal_in_y_thresh = cy >= top_threshold and cy <= bottom_threshold
                else:
                    self.goal_visible = False
                    self.goal_in_thresh = False
                    self.goal_in_y_thresh = False
            else:
                self.goal_visible = False
                self.goal_in_thresh = False
                self.goal_in_y_thresh = False
        else:
            self.goal_visible = False
            self.goal_in_thresh = False
            self.goal_in_y_thresh = False



	#detect west end of field marker
    def _run_detect_boundary_line(self, img, hsv_img, left_threshold, right_threshold):
        cv2.rectangle(img, (left_threshold, -2), (right_threshold, 481), (0,0,255), 2)

        in_range = cv2.inRange(hsv_img, np.array(self.boundary_endpoint_color_range[0]), np.array(self.boundary_endpoint_color_range[1]))
        in_range = cv2.dilate(in_range, None, iterations=5)
        in_range = cv2.erode(in_range, None, iterations=1)

        contours, _ = cv2.findContours(in_range, 1, cv2.CHAIN_APPROX_NONE)
        if len(contours) > 0:
            max_contour = max(contours, key=cv2.contourArea)		#find the largest contour
            max_contour_COG = cv2.moments(max_contour)			#find the center of the largest contour
            _, _, w, h = cv2.boundingRect(max_contour)
            contourArea = w*h
            # print("contourArea: " + str(contourArea))
            cv2.drawContours(img, max_contour, -1, (0, 255, 0), 2)	#draw all the contours on the ROI image
            if contourArea >= contour_max_size:					#only look at contours greater than 7500
                if max_contour_COG != None and max_contour_COG['m00'] != 0:
                    cx = int(max_contour_COG['m10'] / max_contour_COG['m00'])		#find the x value of the center of the contour
                    self.boundary_endpoint_in_thresh = cx >= left_threshold and cx <= right_threshold
                else:
                    self.boundary_endpoint_in_thresh = False
        else:
            self.boundary_endpoint_in_thresh = False

        self.boundary_line_visible = self._color_detected(hsv_img, mining_line_range, 1) #100
        print(self.boundary_line_visible)

    def run(self):
        cv2.namedWindow("Main", cv2.WINDOW_NORMAL)
        cv2.setWindowProperty("Main", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

        camera = PiCamera()
        camera.resolution = (self.width, self.height)
        camera.framerate = 32
        rawCapture = PiRGBArray(camera, size=(self.width, self.height))

        hand_roi_bounds = [[17, 135], [377, 174]]

        threshold_size = 130 # 230
        left_threshold = int((self.width/2)-threshold_size/2)
        right_threshold = int((self.width/2)+threshold_size/2)

        # camera.capture('foo.jpg')

        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            img = frame.array

            hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            cv2.setMouseCallback("Main", mouseCall, hsv_img)

            if (self.current_action == "detect-goal"):
                self._run_detect_goal(img, hsv_img, left_threshold, right_threshold)

            if (self.current_action == "detect-boundary-line"):
                self._run_detect_boundary_line(img, hsv_img, left_threshold, right_threshold)

            # ==== FACE DETECTION ====
            if (self.current_action == "detect-face"):
                cv2.rectangle(img, (left_threshold, -2), (right_threshold, 481), (0,0,255), 2)

                faces = self.face_cascade.detectMultiScale(gray_img, 1.3, 5)
                if len(faces) > 0:
                    for (x,y,w,h) in faces:
                        cv2.rectangle(img, (x,y), (x+w, y+h), (255,0,0), 2)

                    face = faces[0]
                    self.face_bounds = face
                    center_x = int(face[0]+(face[2]/2))

                    self.face_in_thresh = (center_x >= left_threshold and center_x <= right_threshold)

            
            #  ==== ICE IN HAND ====
            if (self.current_action == "detect-ice"):
                hand_x1 = 160
                hand_y1 = 150
                hand_x2 = 450
                hand_y2 = 350
                hand_roi = hsv_img[hand_y1:hand_y2, hand_x1:hand_x2]

                self.pink_ice_in_hand = self._color_detected(hand_roi, pink_range, 20)

                cv2.rectangle(img, (hand_x1, hand_y1), (hand_x2, hand_y2), (0,0,255), 2)

            cv2.imshow("Main", img)

            key = cv2.waitKey(1) & 0xFF	#listen for the screen to be touched to know when to stop the program
            rawCapture.truncate(0)
            if key == ord("q"):		#stop the program from running
                break

robot = Robot()
vision = Vision()
vision.start()

robotActions = RobotActions(robot, vision)
time.sleep(1)

# time.sleep(99999)


robotActions.goto_boundary_line(green_goal_range) #cone_range)
robot.say("Reached mining area")
time.sleep(2)
robot.say("looking for human")

robotActions.goto_human()
robotActions.accept_ice()

# # # # change the size of the contour being looked for to start looking for the scoring box
contour_max_size = 500

robotActions.goto_boundary_line(pink_range)

robot.say("Reached starting area")

robotActions.goto_goal()
robotActions.drop_ice()
