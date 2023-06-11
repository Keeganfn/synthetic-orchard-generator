import numpy as np
import cv2 as cv
im = cv.imread('test.png')
assert im is not None, "file could not be read, check with os.path.exists()"
imgray = cv.cvtColor(im, cv.COLOR_BGR2GRAY)
ret, thresh = cv.threshold(imgray, 0, 255, 0)
cv.imshow("image", thresh)
cv.waitKey(0)
contours, hierarchy = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

final_contours = []
for i in contours:
    print(cv.contourArea(i))
    if cv.contourArea(i) > 100:
        final_contours.append(i)
        rect = cv.boundingRect(i)
        x, y, w, h = rect
        cv.rectangle(im, (x, y), (x+w, y+h), (255, 0, 0), 2)

cv.drawContours(im, final_contours, -1, (0, 255, 0), 2)
cv.imshow("image", im)
cv.waitKey(0)
