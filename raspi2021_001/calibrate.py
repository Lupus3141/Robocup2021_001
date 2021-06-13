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

CUT_LINE = (160, 200, 100, 180)
CUT_GREEN = (115, 155, 100, 140)

last_save_time = time.time()
save_countdown = 10

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	image = frame.array #speichert das aktuelle Bild der Kamera in Variable ab
	image_bgr = image 

	image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) # Konvertiert das Bild zum Christentum
	image = cv2.GaussianBlur(image, ((9, 9)), 2, 2)

	cut_line = image[CUT_LINE[0]:CUT_LINE[1]][CUT_LINE[2]:CUT_LINE[3]]
	cv2.rectangle(image_bgr, (CUT_LINE[0], CUT_LINE[2]), (CUT_LINE[1], CUT_LINE[3]), (0, 0, 255), 2)

	line_avg = cut_line.mean(axis=0).mean(axis=0)
	line_debug_text = "Line:  (" + str(line_avg[0]) + ", " + str(line_avg[1]) + ", " + str(line_avg[2]) + ")"
	cv2.putText(image_bgr, line_debug_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

	cut_green = image[CUT_GREEN[0]:CUT_GREEN[1]][CUT_GREEN[2]:CUT_GREEN[3]]
	cv2.rectangle(image_bgr, (CUT_GREEN[0], CUT_GREEN[2]), (CUT_GREEN[1], CUT_GREEN[3]), (0, 255, 0), 2)

	green_avg = cut_green.mean(axis=0).mean(axis=0)
	green_debug_text = "Green: (" + str(line_avg[0]) + ", " + str(line_avg[1]) + ", " + str(line_avg[2]) + ")"
	cv2.putText(image_bgr, line_debug_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

	save_countdown = save_countdown - time.time() + last_save_time
	cv2.putText(image_bgr, f"{save_countdown:.1f}")

	if(save_countdown <= 0.0):
		# Save values to file
		with open('values.bin', 'wb') as f:
			b = [line_avg[0], line_avg[1], line_avg[2], green_avg[0], green_avg[1], green_avg[2]]
			f.write(bytearray(b))

			print(line_avg)
			print(green_avg)
			# To read:
			with open('values.bin', 'rb') as file:
				line_avg = []
				green_avg = []
				line_avg.append(file.read(1))
				line_avg.append(file.read(1))
				line_avg.append(file.read(1))
				green_avg.append(file.read(1))
				green_avg.append(file.read(1))
				green_avg.append(file.read(1))
			print(line_avg)
			print(green_avg)
			

	cv2.imshow("HSV", image)
	cv2.imshow("BGR; Debug", image_rgb)