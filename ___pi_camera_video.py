# coding: utf-8
# Autor: Ricardo Antonello 
# Site: cv.antonello.com.br
# E-mail: ricardo@antonello.com.br

# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
print(cv2.__version__)
import numpy as np
#from matplotlib import pyplot as plt
#import os
#import sys
#print(sys.executable)

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.resolution = (320, 240)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(320, 240))
 
# allow the camera to warmup
time.sleep(0.1)
 
# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
  # grab the raw NumPy array representing the image, then initialize the timestamp
  # and occupied/unoccupied text
  image = frame.array
  image = image.copy()
  image[10:60,:,:] = (255,0,0)
  # show the frame
  cv2.imshow("Frame", image)
  key = cv2.waitKey(1) & 0xFF
  # clear the stream in preparation for the next frame
  rawCapture.truncate(0)
  # if the `q` key was pressed, break from the loop
  if key == ord("q"):
    break
