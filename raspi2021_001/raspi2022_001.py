#!/usr/bin/env python
# coding=utf-8

# To do:
# 
# Kreuzungen (SKIP verbessern)
# Nach aufgenommender Kugel Ecke finden
# Ausgang finden
# autostart von Linefollowerprogramm
# prüfen, ob auch wirklich eine Kugel aufgenommen wurde
# schnellere baudrate
# raspi übertakten
# Bei Lücke ein Stückchen in die richtige Richtung drehen (ein paar Werte, bevor weiß kam schauen, ob Linienpos rechts oder links war und dann ein Stück koriggieren)
# Dose umfahren und sich dabei nicht von anderen Linien irritieren lassen (neues ROI, ganz links am Kamerabild bzw. einfach alles rechts abschneiden)
# Silber erkennen verbessern
# Lebendes und totes Opfer unterscheiden

from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import argparse
import time
import cv2
import serial
import random
import math
import os

CUT = (0, 320, 140, 192)
CUT_GRN = (50, 270, 110, 192)
CUT_SILVER = (60, 280, 0, 120)
CUT_RESCUEKIT = (50, 270, 120, 170)
CUT_TOP = (120, 200, 60, 120) #extra cut for skip at intersections
CUT_OBSTACLE = (60, 300, 140, 192)

ser = serial.Serial('/dev/ttyAMA0', 9600, timeout = 0.5) #establish serial connenction 

while(not ser.is_open):
	print("Waiting for Serial...")
	time.sleep(0.1)
print("Opened:", ser.name, "aka Teensy 3.6") 

framesTotal = 0 #counter for frames
startTime = time.time() #for FPS calculation at the end of the program
timeWaitet = 0 #because a normal time.sleep in the loop would distort the FPS calculation, the program counts how long has been waited and subtracs it in the final calculation 
lastLinePos = 0 #were was the line in the last frame?
LinePosLastLoop = [0, 0, 0, 0, 0, 0, 0, 0]
lastA2 = 0
pCounter = 0
LineWidthLastLoop = 0
value = 0
gapcounter = 0 #gets increased if no line could be found in a frame
grn_list = []
grn_counter = 0
rescueCounter = 0
rescue = False
mindist = 300 #minRadius for victims
redCnt = 0 #counts how often red/rk has been detected. first time -> Rescuekit second time -> STOP
rescuekitCounter = 0
x = 0
y = 0
r = 0
obstacle = False

########## FUNCTIONS ##########

def DEBUG():
	cv2.imshow("image_rgb", image_rgb)
	cv2.imshow("image_hsv", image)
	cv2.imshow("cut", cut)
	cv2.imshow("cut_green", green)
	cv2.imshow("cut_silber", cut_silver)
	cv2.imshow("rescuekit", rescuekit)

	#cv2.imshow("Konturen gruen", green)
	cv2.setMouseCallback("mouseRGB", mouseRGB)
	cv2.imshow("mouseRGB", image_rgb)
	
def DEBUG_LastLinePos():
	for i in range(8):
		print(f'LinePosLastLoop[{i}] = {LinePosLastLoop[i]:5d}')

def mouseRGB(event, x, y, flags, param): #to adjust colour values eg for green dots
	if event == cv2.EVENT_LBUTTONDOWN: #checks mouse left button down condition
		colorsB = image_rgb[y, x, 0]
		colorsG = image_rgb[y, x, 1]
		colorsR = image_rgb[y, x, 2]
		colors = image_rgb[y, x]
		"""
		print("Red: ", colorsR)
		print("Green: ", colorsG)
		print("Blue: ", colorsB)
		print("BRG Format: ", colors)
		print("Coordinates of pixel: X: ", x,"Y: ", y)
		"""
		colour = np.uint8([[[colorsB, colorsG, colorsR]]])
		colour_hsv = cv2.cvtColor(colour, cv2.COLOR_BGR2HSV)
		print(colour_hsv)



def delay(duration):
	global timeWaitet
	duration = float(duration)
	time.sleep(duration)
	timeWaitet = timeWaitet + duration

def drive(motorLeft, motorRight, duration):
	if int(duration) == 0:
		print("SPEED IS 0, CHANGED TO 1 INSTEAD")
		duration = 1
	send = str(int(motorLeft)) + ':' + str(int(motorRight)) + ':' + str(int(duration))
	print("Send:", send)
	ser.write(send.encode())
	duration = float(duration / 1000.0)
	time.sleep(0.1)
	while True: #waits for the teensy to execute the command
		readData = ser.readline().decode('ascii').rstrip()
		if readData == "1": 
			break

def turnRelative(deg):
	drive(0, 0, deg)

def armDown():
	sendAndWait("armDown")

def armUp():
	sendAndWait("armUp")

def sendAndWait(send): #sends command and waits for receiving the ok
	ser.write(send.encode())
	while True:
		readData = ser.readline().decode('ascii').rstrip()
		if readData == "1":
			break

def distance():
	ser.write(b"dist")
	while True:
		readData = ser.readline().decode('ascii').rstrip()
		if readData != "":
			return int(readData) * 0.075

def toCornerUnload():
	camera = PiCamera()
	camera.resolution = (320, 180) 
	camera.rotation = 0
	camera.framerate = 32
	rawCapture = PiRGBArray(camera, size=(320, 180))

	for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
		image = frame.array
		image = cv2.GaussianBlur(image, ((5, 5)), 2, 2)

		black = cv2.inRange(image, (0, 0, 0), (255, 255, 75))
		if(cv2.countNonZero(black) > 5000):
			sendAndWait("C")
		

		cv2.imshow("Corner out", image)
		rawCapture.truncate(0)
		key = cv2.waitKey(1) & 0xFF
		framesTotalRescue = framesTotalRescue + 1
		if key == ord("q"):
			print("Avg. FPS:", int(framesTotalRescue / (time.time() - startTimeRescue))) #sendet durchsch. Bilder pro Sekunde (FPS)
			camera.close()
			break

def findCorner(pIsWallRight):
	if pIsWallRight == True:
		print("searching for corner with wall right")
		sendAndWait("turnToOrigin")
		ser.write(b'driveToWall')
		turnRelative(90)


	else:
		print("searching for corner with wall left")

def findExit(pIsWallRight): #find green strip in the evacuation zone
	return

def capture():
	camera = PiCamera()
	camera.resolution = (320, 180) 
	camera.rotation = 0
	camera.framerate = 32
	time.sleep(0.5)

	rCapture = PiRGBArray(camera)

	camera.capture(rCapture, format="bgr")
	camera.close()

	return rCapture.array

def checkForCorner():
	image = capture()

	black = cv2.inRange(image, (0, 0, 0), (75, 75, 75))

	return cv2.countNonZero(black) > 10000

def checkForExit():
	image = capture()

	image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	green = cv2.inRange(image, (52, 60, 48), (75, 255, 255)) # TODO: Werte anpassen

	return cv2.countNonZero(green) > 10000 # TODO: Wert anpassen

def rescueVictim():
	image = capture()	
	image = cv2.GaussianBlur(image, ((5, 5)), 2, 2)
	
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

	circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, dp = 1, minDist = 60, param1 = 34, param2 = 24, minRadius = 2, maxRadius = 300)

	if(circles is not None):
		circles = np.round(circles[0, :]).astype("int")
		if(len(circles) > 0):
			x, y, r = circles[0]

			x = int(x)
			y = int(y)

			pos = x - 160

			#cv2.rectangle(image_rgb, (x, y), (x + w, y + h), (50, 50, 200), 2)
			#cv2.putText(image_rgb, str(pos), (x, y + h + 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (50, 50, 200), 2, cv2.LINE_AA)

			#cv2.imshow("image_rgb", image_rgb)

			movement = 0 # movement in motorspeed * milliseconds
			rotation = 0 # rotation in degrees

			if(y < 120):
				ms = (130 - y) * 1.5
				drive(180, 180, ms)
				movement = 180 * ms
			if(y > 150):
				drive(-130, -130, 30)
				movement = -130 * 30
			if(abs(pos) > 10):
				turnRelative(pos / 4)
				rotation = pos / 4
			if(155 > y > 115 and abs(pos) <= 10):
				sendAndWait("grabVictim")
				return (2, movement, rotation)
			return (1, movement, rotation)

	return (0, 0, 0)

D_ONE_TILE = 1025

def rescue():
	#########################################
	#. # 0, 2 						   3, 2 #
	# #										#
	##										# ^
	#										# |
	#										# y
	# ^ angle = 0 							#
	# -> angle = 90 						#
	#										#
	# 0, 0 							   3, 0 #
	#########################################
	#			x -->		

	################## NEU
	print("-------- RESCUE ---------")

	drive(255, 255, 1000)
	distToEntranceWall = distance()
	print(distToEntranceWall)

	x = 0
	y = 0
	angle = 0
	turnRelative(90)
	distToRightWall = distance()
	
	if(distToEntranceWall < 80):
		angle = 90
		if(distToRightWall < 10):
			x = 3
			y = 0
		else:
			x = 0
			y = 0
	else:
		angle = 180
		if(distToRightWall < 10):
			x = 0
			y = 0
		else:
			x = 0
			y = 2

	print(f"We are at ({x}, {y}), angle = {angle}")

	drive(-200, -200, 500)
	sendAndWait("setOrigin")
	drive(200, 200, 250)
	# if(angle == 90):
	# 	turnRelative(-90)
	# 	angle = angle - 90

	num = 1
	if(angle == 90):
		num = 2

	cornerX = 0
	cornerY = 0
	for i in range(3):
		print(num)
		drive(255, 255, D_ONE_TILE * num)
		if(angle == 0):
			cornerY = cornerY + num
		elif(angle == 90):
			cornerX = cornerX + num
		elif(angle == 180):
			cornerY = cornerY - num
		elif(angle == 270):
			cornerX = cornerX - num

		# Search for corner
		if(checkForCorner()):
			print("CORNER")
			break

		if(num == 1):
			num = 2
		elif(num == 2):
			num = 1

		drive(255, 255, D_ONE_TILE)
		drive(255, 255, 300)
		drive(-255, -255, 130)
		turnRelative(-90)
		drive(-255, -255, 300)
		drive(255, 255, 130)
		angle = angle - 90
		if(angle < 0):
			angle = angle + 360

	turnRelative(-45)
	drive(255, 255, 650)
	turnRelative(-90)
	drive(-255, -255, 650)

	sendAndWait("drop")

	sign = 1
	if(angle == 90 or angle == 270):
		sign = -1

	for _ in range(3):
		drive(255, 255, 200)
		turnRelative(90 * sign)
		drive(255, 255, 350)
		turnRelative(-45 * sign)
		drive(255, 255, 150)
		turnRelative(-90 * sign)
		drive(-255, -255, 300)
		drive(255, 255, 150)
		turnRelative(90 * sign)

		ds = 1
		if(angle == 180 or angle == 270):
			ds = -1
		for i in range(9):
			drive(255, 255, D_ONE_TILE)

			if(i == 1 or i == 2):
				#sendAndWait("turnToOrigin")
				turnRelative(90 * ds)
				#AUSRICHTEN
			elif(i == 5 or i == 6):
				turnRelative(-90 * ds)
				#AUSRICHTEN
			
			res = rescueVictim()
			totalMovementX = res[1] # First adjustment is always in x direction
			totalMovementY = 0
			currAngle = res[2]
			while(res[0] == 1):
				res = rescueVictim()
				currAngle = currAngle + res[2]
				totalMovementX = totalMovementX + res[1] * math.cos(currAngle)
				totalMovementY = totalMovementY + res[1] * math.sin(currAngle)
				print("VICTIM")
			if(res[0] == 2):
				print("CAPTURED")
				# Move back according to the recorded movement
				totalMovement = math.sqrt(totalMovementY * totalMovementY + totalMovementX * totalMovementX)
				alpha = math.atan2(totalMovementY, totalMovementX)
				alpha = ((alpha * 360) / (2*3.1415926535)) #convert to dregrees
				print("ALPHA:", alpha)
				print("TOTAL MOVEMENT", totalMovement)
				print("CURRENT ANGLE", currAngle)
				turnRelative(currAngle - alpha)
				drive(-200, -200, totalMovement / 200)
				turnRelative(alpha) # turn back

				# Start going back to corner by going the same path
				j = i
				while(j > 0):
					j = j - 1
					drive(255, 255, D_ONE_TILE)
					if(i == 5 or i == 6):
						turnRelative(90 * ds)
					elif(i == 1 or i == 2):
						turnRelative(-90 * ds)
				# Now we're back at the tile where we started searching from
				turnRelative(90)
				drive(-200, -200, 300)
				drive(200, 200, 150)
				turnRelative(45)
				drive(255, 255, 650)
				turnRelative(-90)
				drive(-255, -255, 400)

				sendAndWait("drop")
				
	# Start searching for exit
	drive(255, 255, 200)
	turnRelative(90 * sign)
	drive(255, 255, 350)
	turnRelative(-45 * sign)
	drive(255, 255, 150)
	# Ausrichten
	turnRelative(-90 * sign)
	drive(-255, -255, 300)
	drive(255, 255, 150)
	turnRelative(90 * sign)

	for i in range(8):
		drive(255, 255, D_ONE_TILE)
		turnRelative(90 * sign)
		if(checkForExit()):
			drive(255, 255, 500)
			return
		turnRelative(-90 * sign)

		if(i == 1 or i == 3 or i == 6):
			turnRelative(-90 * sign)


##############################################################################################
while True:
	camera = PiCamera()
	camera.resolution = (320, 192)
	camera.rotation = 0
	camera.framerate = 32
	rawCapture = PiRGBArray(camera, size=(320, 192))

	turningGreen = 0

	for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
		#if(ser.in_waiting != 0):
		#	s2 = ser.readline()
		#	print("TEENSY_DEBUG: " + str(s2))

		# if(ser.in_waiting != 0):
		# 	s = str(ser.readline())
		# 	print("TEENSY SAID: " + s)
		# 	if("O" in s):
		# 		obstacle = True
		# 		print("OBSTACLE")

		image = frame.array
		image_rgb = image 

		image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) # Konvertiert das Bild zum Christentum
		image = cv2.GaussianBlur(image, ((9, 9)), 2, 2)

		
		A = 30
		if (LinePosLastLoop[0] < -A or LinePosLastLoop[0] > A) and LineWidthLastLoop > 160:
			cut_top = image[CUT_TOP[2]:CUT_TOP[3],CUT_TOP[0]:CUT_TOP[1]]            
			cv2.imshow("cut_top", cut_top)
			#cv2.GaussianBlur(cut_top, ((9, 9)), 2, 2)

			line_top = cv2.inRange(cut_top, (0, 0, 0), (255, 255, 75))

			contours_top, hierarchy_top = cv2.findContours(line_top.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
			if(len(contours_top) > 0):
				#ser.write(b'S')
				#print("SKIP")
				delay(0.4)		
		cut = image[CUT[2]:CUT[3],CUT[0]:CUT[1]]
		cut_grn = image[CUT_GRN[2]:CUT_GRN[3],CUT_GRN[0]:CUT_GRN[1]] 
		cut_silver = image[CUT_SILVER[2]:CUT_SILVER[3],CUT_SILVER[0]:CUT_SILVER[1]]
		cut_rescuekit = image[CUT_GRN[2]:CUT_GRN[3],CUT_GRN[0]:CUT_GRN[1]]
		cut_stop = image[CUT_GRN[2]:CUT_GRN[3],CUT_GRN[0]:CUT_GRN[1]]

		if(turningGreen != 0):
			off = 0
			if(turningGreen == 1):
				off = -60
			else:
				off = 60

			cut_green_stop = image[120:192,(130 + off):(190 + off)]

			green_stop = cv2.inRange(cut_green_stop, (0, 0, 0), (255, 255, 75))
			#contours_green_stop, hierarchy_stop = cv2.findContours(stop.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

			cv2.rectangle(image_rgb, (130 + off, 120), (190 + off, 192), (0, 255, 0), 2)

			print(cv2.countNonZero(green_stop))
			if cv2.countNonZero(green_stop) > 300:	
				print("Finished turning Green")
				turningGreen = 0
				ser.write(b'\nG\n')
				delay(0.2)
				#ser.write(b'G')
				#delay(0.2)
				cv2.putText(image_rgb, "Green end", (65, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 106, 255), 3)



		line = cv2.inRange(cut, (0, 0, 0), (255, 255, 75))
		green = cv2.inRange(cut_grn, (52, 60, 48), (75, 255, 255))
		silber = cv2.inRange(cut_silver, (0, 0, 0), (255, 255, 75))
		rescuekit = cv2.inRange(cut_rescuekit, (119, 200, 25), (125, 255, 150))
		stop = cv2.inRange(cut_rescuekit, (165, 150, 100), (175, 255, 200))



		kernel = np.ones((4, 4), np.uint8)
		green = cv2.erode(green, kernel, iterations=3)
		green = cv2.dilate(green, kernel, iterations=5)

		contours_blk, hierarchy_blk = cv2.findContours(line.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		contours_grn, hierarchy_grn = cv2.findContours(green.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		contours_silver, hierarchy_silver = cv2.findContours(silber.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		contours_rescuekit, hierarchy_rescuekit = cv2.findContours(rescuekit.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		contours_stop, hierarchy_stop = cv2.findContours(stop.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)


		
		linePos = 0
		index = 0
		
		if len(contours_rescuekit) > 0:			
			ser.write(b'RK') #send rescue kit
			delay(1)
			while len(contours_rescuekit) > 0:
				rcapture = PiRGBArray(camera)
				camera.capture(rcapture, format="bgr")
				image = rcapture.array
				image_rgb = image

				image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
				image = cv2.GaussianBlur(image, ((9, 9)), 2, 2)

				rescuekit = cv2.inRange(image, (100, 150, 25), (145, 255, 255))

				kernel = np.ones((4, 4), np.uint8)
				rescuekit = cv2.erode(rescuekit, kernel, iterations=3)
				rescuekit = cv2.dilate(rescuekit, kernel, iterations=5)
				contours_rescuekit, hierarchy_rescuekit = cv2.findContours(rescuekit.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

				if(len(contours_rescuekit) == 0):
					break
				b = cv2.boundingRect(contours_rescuekit[0])
				x, y, w, h = b
				pos = x + w / 2 - 160

				cv2.rectangle(image_rgb, (x, y), (x + w, y + h), (50, 50, 200), 2)
				cv2.putText(image_rgb, str(pos), (x, y + h + 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (50, 50, 200), 2, cv2.LINE_AA)

				cv2.imshow("image_rgb", image_rgb)

				if(y < 120):
					drive(180, 180, (130 - y) * 1.5)
				if(y > 150):
					drive(-130, -130, 30)
				if(abs(pos) > 10):
					turnRelative(pos / 4)
				if(155 > y > 115 and abs(pos) <= 10):
					ser.write(b'grabRescueKit')
					delay(6)
					break
					# drive(-200, -200, 50)
					# turnRelative(180)
					# drive(-200, -200, 85)
					# armDown()
					# armUp()

		if len(contours_stop) > 0:
			b = cv2.boundingRect(contours_stop[0])
			x, y, w, h = b
			if(w * h > 800):
				ser.write(b'STOP')
				print("SEND: STOP")

		### silverdetection: 
		if len(contours_silver) > 0: #black contour > 0 -> no silver
			x_silber, y_silber, w_silber, h_silber = cv2.boundingRect(contours_silver[0]) #make rectangle around contour
			#cv2.rectangle(image_rgb, (x_silber, y_silber), (x_silber + w_silber, y_silber + h_silber), (189, 189, 189), 3) 
			if rescueCounter > 2: #lower rescueCnt since there is a black contour
				rescueCounter = rescueCounter - 3
		
		elif len(contours_silver) == 0: #potential silber
			rescueCounter = rescueCounter + 1
			if rescueCounter > 10: #no black contours for 10 frames -> there must be the evacuation zone
				print("detected silver")
				cv2.putText(image_rgb, "rescue", (65, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 106, 255), 3)
				ser.write(b'Rescue') #sends "Rescue" to the teensy to prove the rescue area with the distance sensor
				read_serial = ser.readline().decode('ascii') 
				if read_serial == '8\r\n': #yep, the distance is between 80cm and 130cm 
					cv2.destroyAllWindows()
					camera.close()
					rescue()
					break
				else:
					print("Teensy said: there can't be the evacuation zone")
					ser.write(str(0/10).encode())
					rescueCounter = 0
		
		if(len(contours_blk) > 0):
			if(len(contours_blk) > 4):
				if(len(contours_silver) == 0):
					print("detected silver")
					cv2.putText(image_rgb, "rescue", (65, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 106, 255), 3)
					ser.write(b'Rescue') #sends "Rescue" to the teensy to prove the rescue area with the distance sensor
					
					read_serial = ser.readline().decode('ascii') 
					if "8" in read_serial: #yep, the distance is between 80cm and 130cm 
						cv2.destroyAllWindows()
						camera.close()
						rescue()
						break
					else:
						print("Teensy said: there can't be the evacuation zone")
						ser.write(str(0/10).encode())
						rescueCounter = 0

			nearest = 1000
			a1 = 0
			a2 = 0
			for i in range(len(contours_blk)):
				b = cv2.boundingRect(contours_blk[i])
				x, y, w, h = b
				a = int(abs(x + w / 2 - 160 - lastLinePos))
				if(len(contours_blk) == 2):
					if(a1 == 0):
						a1 = a
					else:
						a2 = a
				else:
					pCounter = 0

				cv2.rectangle(image_rgb, (x, y + CUT[2] + CUT[0]), (x + w, y + h + CUT[2] + CUT[0]), (0, 106, 255), 2) #rechteck um schwarze Konturen
				if(a < nearest):
					nearest = a
					index = i

			if not (a1 == nearest):
				a1, a2 = a2, a1

			if (abs(lastA2 - a1) > abs(a2 - a1)): # Zweite Kontur nähert sich der ersten an
				pCounter = pCounter + 1

				if(pCounter > 10) and (abs(a2 - a1) < 40):
					print("Ecke erkannt")
					pCounter = 0
					ser.write(b'\nE\n')
			# else:
			# 	pCounter = 0

			lastA2 = a2
			#print(pCounter)
			#pCounter = pCounter - 1
			b = cv2.boundingRect(contours_blk[index])
			x, y, w, h = b
			#print(w)
			LineWidthLastLoop = w
			if(w > 310): #black contours is nearly as big as the whole width of the image -> there must be an intersection 
				cv2.putText(image_rgb, "intersection", (65, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 106, 255), 3)
				ser.write(b'\nS\n')
				delay(0.045)
				print("Send: Skipped")

			linePos = int(x + w / 2 - 160)
			cv2.putText(image_rgb, str(linePos),(linePos + 140, 70), cv2.FONT_HERSHEY_DUPLEX, 2, (0, 106, 255), 2)
			#cv2.line(image_rgb, (linePos + 160, 80), (linePos + 160, 160), (255, 0, 0),2)
			#cv2.line(image_rgb, (0, 110), (319, 110), (255, 0, 0), 2)
			lastLinePos = linePos

			# tg = False
			# if(turningGreen == 1):
			# 	tg = abs(linePos + 10) < 30
			# elif(turningGreen == 2):
			# 	tg = abs(linePos - 10) < 30

			if(obstacle and abs(linePos - 20) < 40):
				print("OBSTACLE")
				obstacle = False
				ser.write(b'\nO\n')
				cv2.putText(image_rgb, "Obstacle end", (65, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 106, 255), 3)
			
		contours_right = False
		contours_left = False   
		if(len(contours_grn) > 0 and len(contours_grn) < 3):
			if(grn_counter <= 0):
				grn_counter = 2
			else:
				if(grn_counter == 1):
					left = 0
					right = 0
					d = False
					s = 0
					for c in grn_list:
						if(c == "L"):
							left = left + 1
							print("L")
						elif(c == "R"):
							right = right + 1
							print("R")
						elif(c == "D"):
							d = True
							print("D")
						elif(c == "S"):
							s = s + 1
							print("S")
					if(d): #deadend
						ser.write(b'D') 
						print("deadend") 
						print("Send: D")
						#delay(1)
					elif(s >= 6):
						#ser.write(b'S')
						#delay(0.2)
						print("Send: S")
					else:
						if(left > right):
							ser.write(b'L')
							turningGreen = 1
							print("Send: L")
							delay(0.5)
						elif(right > left):
							ser.write(b'R')
							turningGreen = 2
							print("Send: R")
							delay(0.5)
					grn_counter = 0
					grn_list.clear()
					#print("List cleared!")
					# for c in grn_list:
					#   print(c)
			check = True
			for i in range(len(contours_blk)):
					b = cv2.boundingRect(contours_blk[index])
					x, y, w, h = b
					if(w > 1000):
						grn_list.append("S")
						check = False

			if(check):
				for i in range(len(contours_grn)):
					b = cv2.boundingRect(contours_grn[i])
					x, y, w, h = b
					cv2.rectangle(image_rgb, (x, y + CUT_GRN[2] + CUT_GRN[0]), (x + w, y + h + CUT_GRN[2] + CUT_GRN[0]), (0, 255, 0), 3) #rectangle around green contours
					a = x + w / 2 - 160 + CUT_GRN[0]
					if(a < linePos):
						contours_left = True
					elif(a > linePos):
						contours_right = True

		else:
			if(grn_counter > 0):
				print("abort")
				grn_counter = 0
				grn_list = []

		if(contours_left and contours_right):
			for i in range(len(contours_grn)):
				b = cv2.boundingRect(contours_grn[i])
				x, y, w, h = b
				#cv2.rectangle(image_rgb, (x, y + CUT_GRN[2]), (x + w, y + h + CUT_GRN[2]), (0, 255, 0), 3)
				a = x + w/2 - 160
				if(a < linePos):
					contours_left = True
				elif(a > linePos):
					contours_right = True

			if(contours_left and contours_right):
				grn_list.append("D")

		elif(contours_left):
			for i in range(len(contours_grn)):
				b = cv2.boundingRect(contours_grn[i])
				x, y, w, h = b
				#cv2.rectangle(image_rgb, (x, y + CUT[3]), (x + w, y + h + CUT[3]), (0, 255, 0), 3)
				a = x + w / 2 - 160
				if(a < linePos):
					contours_left = True
				elif(a > linePos):
					contours_right = True

			if(contours_left):
				grn_list.append("L")
				if(grn_counter == 7):
					grn_list.append("L")

		elif(contours_right):
			for i in range(len(contours_grn)):
				b = cv2.boundingRect(contours_grn[i])
				x, y, w, h = b
				#cv2.rectangle(image_rgb, (x, y + CUT[3]), (x + w, y + h + CUT[3]), (0, 255, 0), 3)
				a = x + w/2 - 160
				if(a < linePos):
					contours_left = True
				elif(a > linePos):
					contours_right = True

			if(contours_right):
				grn_list.append("R")
				if(grn_counter == 7):
					grn_list.append("R")

		else:
			value = str(linePos).encode()
			value = int(float(value))

			if len(contours_blk) == 0: #no black contour
				print("Gapcounter:", gapcounter)
				gapcounter = gapcounter + 1
				if gapcounter > 2:
					if LinePosLastLoop[7] < -20:
						#drehe links
						#ser.write(b'IL') #send gap, turn left
						print("Linepos7:", LinePosLastLoop[7])
					elif LinePosLastLoop[7] > 20:
						#drehe rechts
						print("Linepos7:", LinePosLastLoop[7])
						#ser.write(b'IR') #send gap, turn rigth
					else:
						pass
						#ser.write(b'I') #send gap
				
			else:
				gapcounter = 0
				ser.write(str(linePos / 10).encode()) 

		if(grn_counter > 0):
			grn_counter = grn_counter - 1
		framesTotal = framesTotal + 1

		rawCapture.truncate(0)
		DEBUG()

		LinePosLastLoop[0] = value
		for i in range(1, 8):
			LinePosLastLoop[i] = LinePosLastLoop[i - 1]

		key = cv2.waitKey(1) & 0xFF
		if key == ord("q"):

			print("Avg. FPS:", int(framesTotal / (time.time() - startTime - timeWaitet))) #sendet durchsch. Bilder pro Sekunde (FPS)
			camera.close()
			exit()