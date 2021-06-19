from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import time
import cv2

camera = PiCamera()
camera.resolution = (320, 192)
camera.rotation = 0
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(320, 192))

#CUT_LINE = (160, 200, 100, 180)
CUT_GREEN = (115, 155, 100, 140)

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	image = frame.array #speichert das aktuelle Bild der Kamera in Variable ab
	image_bgr = image 

	#image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) # Konvertiert das Bild zum Christentum
	#image = cv2.GaussianBlur(image, ((9, 9)), 2, 2)

	cut_line = image[160:200, 0:10]
	#cut_line = cut_line[0:40][0:10]
	#cv2.rectangle(image_bgr, (CUT_LINE[0], CUT_LINE[2]), (CUT_LINE[1], CUT_LINE[3]), (0, 0, 255), 2)

	
	cv2.imshow("HSV", image)
	cv2.imshow("BGR; Debug", image_bgr)
	cv2.imshow("CUT", cut_line)
	

	rawCapture.truncate(0)
	key = cv2.waitKey(1) & 0xFF
	if key == ord("q"):

		#print("Avg. FPS:", int(framesTotal / (time.time() - startTime - timeWaitet))) #sendet durchsch. Bilder pro Sekunde (FPS)
		camera.close()
		exit()