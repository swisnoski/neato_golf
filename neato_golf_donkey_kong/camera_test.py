import cv2
import time
import numpy as np


cv2.namedWindow("preview")
cv2.namedWindow("binary_window")
cv2.namedWindow("filled_window")

camera = cv2.VideoCapture(0)

if camera.isOpened(): # try to get the first frame
    rval, frame = camera.read()
else:
    rval = False


def fill_neato(frame):
   
    filled_neato_frame = frame.copy()

    h, w = frame.shape[:2]
    mask = np.zeros((h+2, w+2), np.uint8)
 
    cv2.floodFill(filled_neato_frame, mask, (0,0), 255)
    inv_filled_neato_frame = cv2.bitwise_not(filled_neato_frame)
    im_out = frame | inv_filled_neato_frame

    return im_out


while rval: # while camera is open
    cv2.imshow("preview", frame) # show the last frame

    binary_image = cv2.inRange(frame, (0, 0, 0), (120, 110, 100))
    cv2.imshow("binary_window", binary_image)

    filled_frame = fill_neato(binary_image)
    cv2.imshow("filled_window", filled_frame)

    rval, frame = camera.read() 
    time.sleep(0.05)
    key = cv2.waitKey(20)
    if key == 27: # exit on ESC
        break



