import cv2
import numpy as np

cv2.namedWindow("Main", cv2.WINDOW_NORMAL)

# Static Image
img = cv2.imread("./test-images/line.png", cv2.IMREAD_COLOR)
cv2.imshow("Main", img)
cv2.waitKey(0)
cv2.destroyAllWindows()