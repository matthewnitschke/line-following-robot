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

    MAX_HEADTURN = 7900
    MIN_HEADTURN = 4200
    HEADTURN_INCREMENT = 50

    MAX_HEADTILT = 6750
    MIN_HEADTILT = 5650
    HEADTILT_INCREMENT = 50

    RIGHT_TURN = 6900 #7400
    LEFT_TURN = 5100 #2110

    MOTOR_INCREMENT = 700

    def __init__(self):
        IP = '10.200.22.237'
        PORT = 5010
        self.client = ClientSocket(IP, PORT)
        self.tango = maestro.Controller()

        self._isTurning = False
        self._isMoving = False

        self.headTurn = 6000
        self.headTilt = 6000
        self.motors = 6000
        self.turn = 6000
        self.tango.setTarget(Robot.MOTORS, self.motors)
        self.tango.setTarget(Robot.TURN, self.turn)
        self.tango.setTarget(Robot.HEADTILT, self.headTilt)
        self.tango.setTarget(Robot.HEADTURN, self.headTurn)

        self.headDirection = "left"


    def say(self, text):
        self.client.sendData(text)
        print("Robot Says: " + str(text))

    def look(self):
        if (self.headDirection == "right"):
            self.headRight()
            if (self.headTurn <= Robot.MIN_HEADTURN):
                self.headDirection = "up"
        elif (self.headDirection == "up"):
            self.headUp()
            if (self.headTilt >= Robot.MAX_HEADTILT):
                self.headDirection = "left"
        elif (self.headDirection == "left"):
            self.headLeft()
            if (self.headTurn >= Robot.MAX_HEADTURN):
                self.headDirection = "down"
        elif (self.headDirection == "down"):
            self.headDown()
            if (self.headTilt <= Robot.MIN_HEADTILT):
                self.headDirection = "right"
    
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
        

    def right(self):
        if (not self._isTurning):
            print("turning right")
            self._isTurning = True
            self.turn = Robot.RIGHT_TURN
            self.tango.setTarget(Robot.TURN, self.turn)
    
    def left(self):
        if (not self._isTurning):
            print("turning left")
            self._isTurning = True
            self.turn = Robot.LEFT_TURN
            self.tango.setTarget(Robot.TURN, self.turn)

    def forward(self):
        if (not self._isMoving):
            self._isMoving = True
            self.motors -= Robot.MOTOR_INCREMENT
            self.tango.setTarget(Robot.MOTORS, self.motors)

    def backward(self):
        if (not self._isMoving):
            self._isMoving = True
            self.motors += Robot.MOTOR_INCREMENT
            self.tango.setTarget(Robot.MOTORS, self.motors)
    
    def stop(self):
        self._isTurning = False
        self._isMoving = False
        self.turn = 6000
        self.motors = 6000
        self.tango.setTarget(Robot.TURN, self.turn)
        self.tango.setTarget(Robot.MOTORS, self.motors)

    def isTurning(self):
        return self._isTurning

    def resetHead(self):
        directionToTurn = ""
        if (self.headTurn < 6000):
            directionToTurn = "left"
        else:
            directionToTurn = "right"

        self.headTurn = 6000
        self.headTilt = 6000
        self.tango.setTarget(Robot.HEADTURN, self.headTurn)
        self.tango.setTarget(Robot.HEADTILT, self.headTilt)

        return directionToTurn

class Vision(threading.Thread):

    def __init__(self):
        super(Vision, self).__init__()

        self.width = 0
        self.height = 0

        self.face_cascade = cv2.CascadeClassifier('data/haarcascades/haarcascade_frontalface_default.xml')
        self._faceInFrame = False
        self._faceInThreshold = False
        self._faceBounds = None

    def faceInFrame(self):
        return self._faceInFrame

    def faceInThreshold(self):
        return self._faceInThreshold

    def getFaceBounds(self):
        return self._faceBounds

    def getFrameSize(self):
        return self.width, self.height
    
    def run(self): 
        cv2.namedWindow("Main", cv2.WINDOW_NORMAL)
        cv2.setWindowProperty("Main", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

        camera = PiCamera()
        camera.resolution = (640, 480)				#set the resolution of the camera
        camera.framerate = 32
        rawCapture = PiRGBArray(camera, size=(640, 480))

        lastSeenFace = 0 # keeps track of the last time a face was seen (ms)
        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            img = frame.array

            self.height = img.shape[0]
            self.width = img.shape[1]

            threshold_size = 230
            left_threshold = int((self.width/2)-threshold_size/2)
            right_threshold = int((self.width/2)+threshold_size/2)
            top_threshold = int((self.height/2)-threshold_size/2)
            bottom_threshold = int((self.height/2)+threshold_size/2)

            cv2.rectangle(img, (left_threshold, top_threshold), (right_threshold, bottom_threshold), (0,0,255), 2)

            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            faces = self.face_cascade.detectMultiScale(gray, 1.3, 5) # 1.3

            if len(faces) > 0:
                for (x,y,w,h) in faces:
                    cv2.rectangle(img, (x,y), (x+w, y+h), (255,0,0), 2)
                
                self._faceInFrame = True

                face = faces[0]
                self._faceBounds = face
                center_x = int(x+(face[2]/2))
                center_y = int(y+(face[1]/2))

                cv2.line(img, (center_x, 0), (center_x, self.height), (0,0,255), 1)
                cv2.line(img, (0, center_y), (self.width, center_y), (0,0,255), 1)

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

vision = Vision()
vision.start()

robot = Robot()

MISSING_FACE_WINDOW = 1000*15 # how long to wait before start searching for new face (ms)

lastSeenFace = 0
lookingForHuman = True
turningToHuman = False
movingToHuman = False

directionToTurn = ""



running = True
while running:
    now = currentTimeMills()

    inThreshold = (lastSeenFace + MISSING_FACE_WINDOW >= now)
    if (vision.faceInFrame()):
        if (not inThreshold):
            robot.say("hello human")
            time.sleep(1)
            lookingForHuman = False
            turningToHuman = True
            movingToHuman = False
            directionToTurn = robot.resetHead()
            time.sleep(1)
        lastSeenFace = now
    else:
        if (not inThreshold):
            lookingForHuman = True
            movingToHuman = False
            turningToHuman = False
            if (robot.isTurning()):
                robot.stop()

    if lookingForHuman:
        robot.look()
    
    if turningToHuman:
        if directionToTurn == "right":
            robot.right()
        
        if directionToTurn == "left":
            robot.left()

        if (vision.faceInThreshold()):
            print("Face in thresh")
            robot.stop()
            turningToHuman = False
            movingToHuman = True

        time.sleep(.5)
        robot.stop()
        time.sleep(.5)
    
    if movingToHuman:
        maxFaceArea = 25000
        minFaceArea = 10000

        if (vision.faceInFrame()):
            (_, _, face_w, face_h) = vision.getFaceBounds()
            faceArea = face_w * face_h

            print(faceArea)

            if (faceArea > maxFaceArea):
                robot.backward()
            elif (faceArea < minFaceArea):
                robot.forward()
            else:
                # we are in the face threshold, kill loop
                robot.stop()
                running = False

    time.sleep(.1)

# Keep the face within the robots vision
frame_w, frame_h = vision.getFrameSize()
face_track_thresh = 160
min_x = (frame_w/2) - (face_track_thresh/2)
max_x = (frame_w/2) + (face_track_thresh/2)
min_y = (frame_h/2) - (face_track_thresh/2)
max_y = (frame_h/2) + (face_track_thresh/2)

frame_center_x = frame_w/2
frame_center_y = frame_h/2
while True:
    if (vision.faceInFrame()):
        (x, y, w, h) = vision.getFaceBounds()

        face_center_x = (x + (w/2))
        face_center_y = (y + (h/2))

        if (face_center_x < min_x):
            robot.headLeft()
        elif (face_center_x > max_x):
            robot.headRight()
        
        if (face_center_y < min_y):
            robot.headUp()
        elif (face_center_y > max_y):
            robot.headDown()
    time.sleep(.1)

robot.stop()
cv2.destroyAllWindows()

