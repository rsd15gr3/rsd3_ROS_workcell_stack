import cv2

im = cv2.imread("red.png", 0)
cv2.imshow("image", im)
cv2.waitKey(0)
cv2.destroyAllWindows()