import cv2
import numpy as np

CENTER_THRESH_SIZE = 80

def robotRight():
    print("ROBOT TURNING RIGHT")

def robotForward():
    print("ROBOT GOING FORWARD")

def robotLeft():
    print("ROBOT TURNING LEFT")

def mouseCall(evt, x, y, flags, pic):
    global robotRunning
    if evt == cv2.EVENT_LBUTTONDOWN:
        if x > 0 and x < 100:
            if y > len(pic[y]) - 100 and y < len(pic[y]):
                robotRunning = True
        # print(pic[y][x])

cv2.namedWindow("Main", cv2.WINDOW_NORMAL)

detectionRunning = True
robotRunning = False
while detectionRunning:
    img = cv2.imread("./test-images/breadcrumb.png", cv2.IMREAD_COLOR)
    
    cv2.setMouseCallback("Main", mouseCall, img)

    height = img.shape[0]
    width = img.shape[1]

    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    minHSV = np.array([0, 30, 209])
    maxHSV = np.array([57, 255, 255])

    color_detected = cv2.inRange(hsv_img, minHSV, maxHSV)

    contours, _ = cv2.findContours(color_detected, 1, cv2.CHAIN_APPROX_NONE)
    if len(contours) > 0:
        max_contour = max(contours, key=cv2.contourArea)
        max_contour_COG = cv2.moments(max_contour)
        cx = int(max_contour_COG['m10'] / max_contour_COG['m00'])
        cy = int(max_contour_COG['m01'] / max_contour_COG['m00'])

        cv2.line(img, (cx, 0), (cx, height), (255, 0, 0), 2)
        cv2.line(img, (0, cy), (width, cy), (255, 0, 0), 2)
        
        cv2.drawContours(img, contours, -1, (0,255,0), 2)

        leftBound = (width/2)-CENTER_THRESH_SIZE/2
        rightBound = (width/2)+CENTER_THRESH_SIZE/2
        cv2.rectangle(img, (leftBound, -5), (rightBound, height+5), (0,0,255), 2)

        if cx >= rightBound:
            robotRight()
        if cx < rightBound and cx > leftBound:
            robotForward()
        if cx <= leftBound:
            robotLeft()
    else:
        cv2.circle(img, (width - 50, height - 50), 40, (0,0,255), -1)
    
    if not robotRunning:
        cv2.circle(img, (50, height - 50), 40, (0,0,255), -1)


cv2.imshow("Main", img)
cv2.waitKey(0)
cv2.destroyAllWindows()