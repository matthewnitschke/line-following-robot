import cv2
import time
from picamera.array import PiRGBArray
from picamera import PiCamera
import maestro
import threading
from client import ClientSocket

def currentTimeMills():
    return int(round(time.time() * 1000))

class Robot:
    MOTORS = 1
    TURN = 2
    BODY = 0
    HEADTILT = 4
    HEADTURN = 3

    MAX_HEADTURN = 7900             #maximum value of the servo to turn the head to the right
    MIN_HEADTURN = 4200             #minimum value of the servo to turn the head to the left
    HEADTURN_INCREMENT = 50         #amount to turn per revolution

    MAX_HEADTILT = 6750             #maximum value of the servo to move the head up
    MIN_HEADTILT = 5650             #minimum value of the servo to move the head dwn
    HEADTILT_INCREMENT = 50

    RIGHT_TURN = 6900 #7400         #servo motor speed to get the robot to turn right
    LEFT_TURN = 5100 #2110          #servo motor speed to get the robot to turn left

    MOTOR_INCREMENT = 700

    def __init__(self):
        IP = '10.200.22.237'        #ip address of Tango phone
        PORT = 5010
        self.client = ClientSocket(IP, PORT)
        self.tango = maestro.Controller()

        self._isTurning = False
        self._isMoving = False

        #initializes motors and directions to center
        self.headTurn = 6000
        self.headTilt = 6000
        self.motors = 6000
        self.turn = 6000
        self.tango.setTarget(Robot.MOTORS, self.motors)
        self.tango.setTarget(Robot.TURN, self.turn)
        self.tango.setTarget(Robot.HEADTILT, self.headTilt)
        self.tango.setTarget(Robot.HEADTURN, self.headTurn)

        self.headDirection = "left"

#method for having the robot speak
    def say(self, text):
        self.client.sendData(text)          #sends the message to the phone to have the phone commence speaking
        print("Robot Says: " + str(text))

#methods for moving the head around when no human has been detected yet (searching for a human)
    def look(self):
        if (self.headDirection == "right"):
            self.headRight()
            if (self.headTurn <= Robot.MIN_HEADTURN):       #only allows the head to move so far
                self.headDirection = "up"                   #after moving to the right the head moves up
        elif (self.headDirection == "up"):
            self.headUp()
            if (self.headTilt >= Robot.MAX_HEADTILT):       #only lest the head move up so far
                self.headDirection = "left"                 #after moving the head up the head starts moving left
        elif (self.headDirection == "left"):
            self.headLeft()
            if (self.headTurn >= Robot.MAX_HEADTURN):       #only allows the head to turn left so far
                self.headDirection = "down"                 #after turning left the head moves down
        elif (self.headDirection == "down"):
            self.headDown()
            if (self.headTilt <= Robot.MIN_HEADTILT):       #only allows the head to go down so far
                self.headDirection = "right"                #after moving the head down the head starts moving right
                
#sets the motor speeds and increments the movements for the head
    def headRight(self):
        self.headTurn -= Robot.HEADTILT_INCREMENT
        self.tango.setTarget(Robot.HEADTURN, self.headTurn)
    
    def headLeft(self):
        self.headTurn += Robot.HEADTILT_INCREMENT
        self.tango.setTarget(Robot.HEADTURN, self.headTurn)

    def headUp(self):
        self.headTilt += Robot.HEADTILT_INCREMENT
        self.tango.setTarget(Robot.HEADTILT, self.headTilt)
    
    def headDown(self):
        self.headTilt -= Robot.HEADTILT_INCREMENT
        self.tango.setTarget(Robot.HEADTILT, self.headTilt)
        
#sets the turning motors speed and direction
    def right(self):
        if (not self._isTurning):           #if the robot is not already turning make it start turning
            print("turning right")
            self._isTurning = True
            self.turn = Robot.RIGHT_TURN
            self.tango.setTarget(Robot.TURN, self.turn)
    
    def left(self):
        if (not self._isTurning):               #if the robot is not already turning make it start turning
            print("turning left")
            self._isTurning = True
            self.turn = Robot.LEFT_TURN
            self.tango.setTarget(Robot.TURN, self.turn)

    def forward(self):
        if (not self._isMoving):            #if the robot is not already moving forward make it start moving forward
            self._isMoving = True
            self.motors -= Robot.MOTOR_INCREMENT
            self.tango.setTarget(Robot.MOTORS, self.motors)

    def backward(self):
        if (not self._isMoving):                #if the robot is not already moving backward make it start moving backward
            self._isMoving = True
            self.motors += Robot.MOTOR_INCREMENT
            self.tango.setTarget(Robot.MOTORS, self.motors)
    
    def stop(self):                             #set the motors back to center to stop moving
        self._isTurning = False
        self._isMoving = False
        self.turn = 6000
        self.motors = 6000
        self.tango.setTarget(Robot.TURN, self.turn)
        self.tango.setTarget(Robot.MOTORS, self.motors)

    def isTurning(self):
        return self._isTurning

 #resets the head back to being centered on the robot then the robot turns its whole body back to being centered on the human
    def resetHead(self):                        #sets the head back to centered on the robot
        directionToTurn = ""
        if (self.headTurn < 6000):              #tells the robot which direction to get centered back on the human
            directionToTurn = "left"
        else:
            directionToTurn = "right"

        self.headTurn = 6000
        self.headTilt = 6000
        self.tango.setTarget(Robot.HEADTURN, self.headTurn)
        self.tango.setTarget(Robot.HEADTILT, self.headTilt)

        return directionToTurn

#separate thread used for detecting the face of the human, drawing a blue box around the face and tracking
#whether the face has been moved into the threshold of being centered in the screen or not
class Vision(threading.Thread):

    def __init__(self):
        super(Vision, self).__init__()

        self.width = 0
        self.height = 0

#detects faces
        self.face_cascade = cv2.CascadeClassifier('data/haarcascades/haarcascade_frontalface_default.xml')
        self._faceInFrame = False
        self._faceInThreshold = False
        self._faceBounds = None

#keeps track of when the face is in the screen
    def faceInFrame(self):
        return self._faceInFrame

#keeps track of when the face is in the threshold of the center of the screen
    def faceInThreshold(self):
        return self._faceInThreshold

#determines the size of the face so it can know whether to move forward or backward to be at the set 
#distance away from the robot
    def getFaceBounds(self):
        return self._faceBounds

#gets the size of the screen
    def getFrameSize(self):
        return self.width, self.height
 
#runs the vision thread
    def run(self): 
        cv2.namedWindow("Main", cv2.WINDOW_NORMAL)                  #create a window to draw the image on
        cv2.setWindowProperty("Main", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)       #sets the window to full screen

        camera = PiCamera()
        camera.resolution = (640, 480)				#set the resolution of the camera
        camera.framerate = 32
        rawCapture = PiRGBArray(camera, size=(640, 480))

        lastSeenFace = 0 # keeps track of the last time a face was seen (ms)
        #continuously get frames from the camera
        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            img = frame.array

            self.height = img.shape[0]
            self.width = img.shape[1]

#determines the threshold size for keeping the face in the center of the screen
            threshold_size = 230
            left_threshold = int((self.width/2)-threshold_size/2)
            right_threshold = int((self.width/2)+threshold_size/2)
            top_threshold = int((self.height/2)-threshold_size/2)
            bottom_threshold = int((self.height/2)+threshold_size/2)

#draws a rectangle around the threshold area
            cv2.rectangle(img, (left_threshold, top_threshold), (right_threshold, bottom_threshold), (0,0,255), 2)

            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)            #turns image to gray scale
            faces = self.face_cascade.detectMultiScale(gray, 1.3, 5) # 1.3      #detects the face

#draws a rectangle around the face
            if len(faces) > 0:
                for (x,y,w,h) in faces:
                    cv2.rectangle(img, (x,y), (x+w, y+h), (255,0,0), 2)
                
                self._faceInFrame = True        #face is visible on the screen

#get the center of the face to use for guiding the robot to put the face in the center of the screen
                face = faces[0]
                self._faceBounds = face
                center_x = int(x+(face[2]/2))
                center_y = int(y+(face[1]/2))

#draw "cross hairs" on the center of the face
                cv2.line(img, (center_x, 0), (center_x, self.height), (0,0,255), 1)
                cv2.line(img, (0, center_y), (self.width, center_y), (0,0,255), 1)

#tracks when the face is in the center of the screens threshold
                if (center_x >= left_threshold and center_x <= right_threshold):
                    self._faceInThreshold = True
                else:
                    self._faceInThreshold = False
                
            else:
                self._faceInFrame = False
                self._faceBounds = None

            cv2.imshow("Main", img)

            key = cv2.waitKey(1) & 0xFF	#listen for the screen to be touched to know when to stop the program
            rawCapture.truncate(0)
            if key == ord("q"):		#stop the program from running
                break

vision = Vision()           #start the thread for the vision part
vision.start()

robot = Robot()

MISSING_FACE_WINDOW = 1000*15 # how long to wait before start searching for new face (ms)

lastSeenFace = 0        #initalize last seen face to 0
lookingForHuman = True      #initialize robot at the start looking for human part
turningToHuman = False
movingToHuman = False

directionToTurn = ""



running = True
#commands for the robot to do while looking for a human, talking to the human, turning to the human and moving to the human
while running:
    now = currentTimeMills()

    inThreshold = (lastSeenFace + MISSING_FACE_WINDOW >= now)       #keeps track of when a human was last seen
    if (vision.faceInFrame()):              #robot has detected a face in the area of the screen
        if (not inThreshold):               #human is not centered in the screen
            robot.say("hello human")        #robot speaks
            time.sleep(1)                   #allows time for computer processing
            lookingForHuman = False         #no longer looking for a human
            turningToHuman = True           #need to turn to the human to get them centered in the screen
            movingToHuman = False           #turn to human then move to human
#keep track of where the human is so when head goes back to center of body the robot knows which direction to turn to find the human again
            directionToTurn = robot.resetHead()         
            time.sleep(1)                   #allows time for computer processing
        lastSeenFace = now                  #resets the time of when a face has last been seen
    else:                                   #human has not been detected. keep looking for a human
        if (not inThreshold):
            lookingForHuman = True
            movingToHuman = False
            turningToHuman = False
            if (robot.isTurning()):
                robot.stop()

    if lookingForHuman:                     #move the head around when looking for a human
        robot.look()
    
    if turningToHuman:                      #turn to the human based off the direction received when resetting the head to center
        if directionToTurn == "right":
            robot.right()
        
        if directionToTurn == "left":
            robot.left()

        if (vision.faceInThreshold()):      #once face is in the center threshold stop turning and start moving towards the human
            print("Face in thresh")
            robot.stop()
            turningToHuman = False
            movingToHuman = True

        time.sleep(.5)                      #allows time for computer processing
        robot.stop()
        time.sleep(.5)
    
    #when the robot is moving toward the human, keep track of the area of the face to know when to move forward or backward
    if movingToHuman:                       
        maxFaceArea = 25000         #the largest the face should be to have the robot close enough to the human
        minFaceArea = 10000         #the farthest away the face should be to be far enough away from the human

        if (vision.faceInFrame()):              #face has been detected
            (_, _, face_w, face_h) = vision.getFaceBounds()
            faceArea = face_w * face_h          #get the area of the face

            print(faceArea)

            if (faceArea > maxFaceArea):        #if the area of the face is larger than the max threshold the face is too close. back up
                robot.backward()
            elif (faceArea < minFaceArea):      #if the area of the face is smaller than the min threshold the face is too far. close in
                robot.forward()
            else:
                # we are in the face threshold, kill loop
                robot.stop()
                running = False

    time.sleep(.1)              #allows time for computer processing

# Keep the face within the robots vision
frame_w, frame_h = vision.getFrameSize()
face_track_thresh = 160
min_x = (frame_w/2) - (face_track_thresh/2)
max_x = (frame_w/2) + (face_track_thresh/2)
min_y = (frame_h/2) - (face_track_thresh/2)
max_y = (frame_h/2) + (face_track_thresh/2)

frame_center_x = frame_w/2
frame_center_y = frame_h/2
#continue moving the head around to keep the human's face in the threshold of the screen
while True:
    if (vision.faceInFrame()):
        (x, y, w, h) = vision.getFaceBounds()

        face_center_x = (x + (w/2))
        face_center_y = (y + (h/2))

        if (face_center_x < min_x):         #move the head left if the face moved off to the left of the robot
            robot.headLeft()
        elif (face_center_x > max_x):       #move the head right if the face moved off to the right of the robot
            robot.headRight()
        
        if (face_center_y < min_y):         #move the head up if the face moves above the robot
            robot.headUp()
        elif (face_center_y > max_y):       #move the head down if the face moves lower than the robot head
            robot.headDown()
    time.sleep(.1)                          #allows time for computer processing

robot.stop()
cv2.destroyAllWindows()

