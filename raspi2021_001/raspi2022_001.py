#!/usr/bin/env python
# coding=utf-8

# To do:
# 
# Ecke und Ausgang RICHTIG finden und nicht premappen xD
# Nach dem Finden des Rescuekits richtig ausrichten und dann erst aufnehmen
# raspi kühler (aktiv)
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
import os

CUT = (50, 270, 140, 192)
CUT_GRN = (50, 270, 70, 192)
CUT_SILVER = (0, 100, 0, 180)
CUT_RESCUEKIT = (50, 270, 120, 170)
CUT_TOP = (120, 200, 40, 110) #extra cut for skip at intersections

ser = serial.Serial('/dev/ttyAMA0', 9600, timeout = 2) #establish serial connenction 

while(not ser.is_open):
	print("Waiting for Serial...")
	time.sleep(0.1)
print("Opened:", ser.name, "aka Teensy 3.6") 

framesTotal = 0 #counter for frames
startTime = time.time() #for FPS calculation at the end of the program
timeWaitet = 0 #because a normal time.sleep in the loop would distort the FPS calculation, the program counts how long has been waited and subtracs it in the final calculation 
lastLinePos = 0 #were was the line in the last frame?
LinePosLastLoop = [0, 0, 0, 0, 0, 0, 0, 0]
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

########## FUNCTIONS ##########

def DEBUG():
	cv2.imshow("image_rgb", image_rgb)
	cv2.imshow("image_hsv", image)
	cv2.imshow("cut", cut)
	cv2.imshow("cut_green", cut_grn)
	#cv2.imshow("cut_silber", cut_silver)
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
	send = str(motorLeft) + ':' + str(motorRight) + ':' + str(duration)
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

def findCorner(pIsWallRight):
	if pIsWallRight == True:
		print("searching for corner with wall right")
		sendAndWait("driveToBlackCornerAndSaveVictim")
	else:
		print("searching for corner with wall left")

def findExit(pIsWallRight): #find green strip in the evacuation zone
	"""
	time.sleep(0.5)
	print("1")
	camera = PiCamera()
	camera.resolution = (320, 180)
	camera.rotation = 0
	camera.framerate = 32
	rawCapture = PiRGBArray(camera, size=(320, 180))
	print("2")
	"""

	if pIsWallRight == True:
		print("searching for exit with wall right")
		for i in range(3):
			drive(255, 255, 200)
		#camera.close()
		sendAndWait("exit")
		return
	else:
		print("searching for exit with wall left")
		return
		
		"""
		for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
			print("3")			
			image = frame.array
			image = image[50:270][50:192]           
			image_rgb = image

			image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) #convert brg to hsv
			image = cv2.GaussianBlur(image, ((15, 15)), 2, 2)


			green = cv2.inRange(image, (30, 20, 20), (100, 255, 255))

			contours_grn, hierarchy_grn = cv2.findContours(green.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
			
			print(len(contours_grn))
			if(len(contours_grn) > 0):
				cv2.imshow("Exit", image_rgb)
				cv2.putText(image_rgb, "Exit", (110, 60), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 106, 255), 3)
				drive(130, 130, 900)
				drive(150, 0, 300)
				camera.close()
				cv2.destroyAllWindows()
				sendAndWait("exit")
				print("received exit")
				return
			else:
				drive(255, 255, 50)
			cv2.drawContours(image_rgb, contours_grn, -1, (0, 106, 255), 3)
			cv2.imshow("Exit", image_rgb)
			rawCapture.truncate(0)
			key = cv2.waitKey(1) & 0xFF
			if key == ord("q"):
				break
		"""
def rescue():
	noVictim = 0 #counter for frames without vitim, if x frames without one -> turn a bit around
	turnCnt = 0 #how many degrees has the robot turned
	vicitmsSaved = 0 #how many victims have been saved?
	fullRotationCounter = 0 

	camera = PiCamera()
	camera.resolution = (320, 180) 
	camera.rotation = 0
	camera.framerate = 32
	rawCapture = PiRGBArray(camera, size=(320, 180))
	print("Rescue program started")
	time.sleep(1)
	framesTotalRescue = 0 
	startTimeRescue = time.time()

	isWallRight = True #were is the wall in the evacuation zone?
	uselessCnt = 0
	drive(200, 255, 1000)
	turnRelative(-80)
	drive(-255, -255, 500)
	sendAndWait("setOrigin") #save absolute position to come back to it afterwards
	drive(255, 255, 350)
	turnRelative(90)
	drive(255, 255, 1000)
	turnRelative(-45)
	drive(255, 255, 300)
	turnRelative(-90)
	drive(-255, -255, 500)  
	armDown()
	armUp()
	drive(255, 255, 1300)
	sendAndWait("turnToOrigin")
	drive(255, 255, 500)
	
	for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
		image = frame.array
		image = cv2.GaussianBlur(image, ((5, 5)), 2, 2)
		
		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

		circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, dp = 1, minDist = 60, param1 = 34, param2 = 24, minRadius = 2, maxRadius = 300)

		# ensure at least some circles were found
		if circles is not None:
			if noVictim > 5: #victim in the current frame -> lower noVictim counter
				noVictim = noVictim - 5
			else:
				noVictim = 0
			# convert the (x, y) coordinates and radius of the circles to integers
			circles = np.round(circles[0, :]).astype("int")
			# loop over the (x, y) coordinates and radius of the circles
			for (x, y, r) in circles:
				#print(y) #y pos of victim
				# draw the circle in the output image, then draw a rectangle
				cv2.circle(image, (x, y), r, (255, 255, 0), 4)
				#victimColor = image[y, x, 0] + image[y, x, 1] + image[y, x, 2]
				#print("Color of victim centre:", victimColor)
				cv2.rectangle(image, (x - 5, y - 5), (x + 5, y + 5), (0, 0, 255), -1)
				ballPosition = x - 160
				if ballPosition > -7 and ballPosition < 7: #Victim is horizontal aligned
					print(y)
					if y > 120 and y < 140: #turn around and grap ball
						drive(-150, -150, 30)
						turnRelative(180)
						armDown()
						armUp()
						#turn to orogin and search for the black corner
						sendAndWait("turnToOrigin")
						findCorner(isWallRight)	
						drive(255, 255, 1500) #don't search at the same place as before				
						sendAndWait("turnToOrigin")
						turnRelative(90)
						drive(255, 255, 2000)
						drive(-255, -255, 80)
						turnRelative(-70)
						drive(255, 255, 200)
						turnRelative(-15)
						camera.close()
						cv2.destroyAllWindows()
						findExit(isWallRight)
						return
					elif y > 170:
						drive(-255, -255, 80)
					elif y > 115:
						drive(-255, -255, 10)
					elif y < 90:
						drive(255, 255, 40)
					elif y < 140:
						drive(255, 255, 10)
				elif ballPosition <= -7 and ballPosition >= -25:
					drive(-150, 150, 30)
				elif ballPosition <= -25:
					drive(-255, 255, 80)

				elif ballPosition >= 7 and ballPosition <= 25:
					drive(150, -150, 30)
				elif ballPosition >= 25:
					drive(255, -255, 80)
				cv2.putText(image, str(ballPosition), (100, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 3)
		else:
			noVictim = noVictim + 1 #one frame without victim -> increase counter
			if noVictim >= 10: #No victim for 10 frames -> turn a bit
				if turnCnt < 360:
					if fullRotationCounter == 0:
						turnRelative(45)
						turnCnt = turnCnt + 45
					elif fullRotationCounter == 1:
						drive(255, 255, 500)
						turnRelative(70)
						turnCnt = turnCnt + 70
					else:
						uselessCnt = uselessCnt + 1
						if uselessCnt == 1:
							turnRelative(180)
						drive(255, 255, 500)
						turnRelative(-70)
						turnCnt = turnCnt + 70
				else:
					print("turned full 360 degs")
					#sendAndWait("dreheZuUrsprung")
					fullRotationCounter = fullRotationCounter + 1
					turnCnt = 0

		cv2.imshow("Kugel output", image)
		rawCapture.truncate(0)
		key = cv2.waitKey(1) & 0xFF
		framesTotalRescue = framesTotalRescue + 1
		if key == ord("q"):
			print("Avg. FPS:", int(framesTotalRescue / (time.time() - startTimeRescue))) #sendet durchsch. Bilder pro Sekunde (FPS)
			camera.close()
			break
	print("Rescue Program stopped")
##############################################################################################
while True:
	camera = PiCamera()
	camera.resolution = (320, 192)
	camera.rotation = 0
	camera.framerate = 32
	rawCapture = PiRGBArray(camera, size=(320, 192))

	for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
		image = frame.array
		image_rgb = image 

		image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) # Konvertiert das Bild zum Christentum
		image = cv2.GaussianBlur(image, ((9, 9)), 2, 2)

		
		"""
		A = 30
		if (LinePosLastLoop[0] < -A or LinePosLastLoop[0] > A) and LineWidthLastLoop > 100:
			cut_top = image[CUT_TOP[0]:CUT_TOP[1]][CUT_TOP[2]:CUT_TOP[3]]            
			cv2.imshow("cut_top", cut_top)
			#cv2.GaussianBlur(cut_top, ((9, 9)), 2, 2)

			line_top = cv2.inRange(cut_top, (0, 0, 0), (255, 255, 75))

			contours_top, hierarchy_top = cv2.findContours(line_top.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
			if(len(contours_top) > 0):
				ser.write(b'S')
				print("SKIP")
				delay(0.05)
		"""
		cut = image[CUT[0]:CUT[1]][CUT[2]:CUT[3]]
		cut_grn = image[CUT_GRN[0]:CUT_GRN[1]][CUT_GRN[2]:CUT_GRN[3]] 
		cut_silver = image[CUT_SILVER[0]:CUT_SILVER[1]][CUT_SILVER[2]:CUT_SILVER[3]]
		cut_rescuekit = image[CUT_GRN[0]:CUT_GRN[1]][CUT_GRN[2]:CUT_GRN[3]]
		cut_stop = image[CUT_GRN[0]:CUT_GRN[1]][CUT_GRN[2]:CUT_GRN[3]]
		
		cv2.GaussianBlur(cut_silver, ((9, 9)), 2, 2) #cut to detect silver

		line = cv2.inRange(cut, (0, 0, 0), (255, 255, 75)) 
		green = cv2.inRange(cut_grn, (60, 140, 48), (74, 210, 80))
		silber = cv2.inRange(cut_silver, (0, 0, 0), (255, 255, 75))
		rescuekit = cv2.inRange(cut_rescuekit, (119, 200, 25), (125, 255, 150))
		stop = cv2.inRange(cut_rescuekit, (165, 150, 100), (175, 255, 200))

		kernel = np.ones((4, 4), np.uint8)
		green = cv2.erode(green, kernel, iterations=7)
		green = cv2.dilate(green, kernel, iterations=11)

		contours_blk, hierarchy_blk = cv2.findContours(line.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		contours_grn, hierarchy_grn = cv2.findContours(green.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		contours_silver, hierarchy_silver = cv2.findContours(silber.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		contours_rescuekit, hierarchy_rescuekit = cv2.findContours(rescuekit.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		contours_stop, hierarchy_stop = cv2.findContours(stop.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		
		linePos = 0
		index = 0
		#print("Len contours red:", len(contours_stop))

		if len(contours_rescuekit) > 0:	
			pass	
			#ser.write(b'A')	
			#print("SEND: A")
			#time.sleep(0.5)
		if len(contours_stop) > 4:
			ser.write(b'STOP')
			print("SEND: STOP")

		### silverdetection:
		if len(contours_silver) > 0: #black contour > 0 -> no silver
			x_silber, y_silber, w_silber, h_silber = cv2.boundingRect(contours_silver[0]) #make rectangle around contour
			#cv2.rectangle(image_rgb, (x_silber, y_silber), (x_silber + w_silber, y_silber + h_silber), (189, 189, 189), 3) 
			if rescueCounter > 2: #lower rescueCnt since there is a black contour
				rescueCounter = rescueCounter - 3
		
		if len(contours_silver) == 0: #potential silber
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
			nearest = 1000
			for i in range(len(contours_blk)):
				b = cv2.boundingRect(contours_blk[i])
				x, y, w, h = b
				a = int(abs(x + w / 2 - 160 - lastLinePos))
				cv2.rectangle(image_rgb, (x, y + CUT[2] + CUT[0]), (x + w, y + h + CUT[2] + CUT[0]), (0, 106, 255), 2) #rechteck um schwarze Konturen
				if(a < nearest):
					nearest = a
					index = i

			b = cv2.boundingRect(contours_blk[index])
			x, y, w, h = b
			#print(w)
			LineWidthLastLoop = w
			if(w > 280): #black contours is nearly as big as the whole width of the image -> there must be an intersection 
				cv2.putText(image_rgb, "intersection", (65, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 106, 255), 3)
				ser.write(b'S')
				print("Send: Skipped")

			linePos = int(x + w / 2 - 160)
			cv2.putText(image_rgb, str(linePos),(linePos + 140, 70), cv2.FONT_HERSHEY_DUPLEX, 2, (0, 106, 255), 2)
			#cv2.line(image_rgb, (linePos + 160, 80), (linePos + 160, 160), (255, 0, 0),2)
			#cv2.line(image_rgb, (0, 110), (319, 110), (255, 0, 0), 2)
			lastLinePos = linePos

		contours_right = False
		contours_left = False   
		if(len(contours_grn) > 0 and len(contours_grn) < 3):
			if(grn_counter <= 0):
				grn_counter = 6
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
							print("Send: L")
							#delay(0.5)
						elif(right > left):
							ser.write(b'R')
							print("Send: R")
							#delay(0.5)
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
					a = x + w / 2 - 160
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
						ser.write(b'IL') #send gap, turn left
						print("Linepos7:", LinePosLastLoop[7])
					elif LinePosLastLoop[7] > 20:
						#drehe rechts
						print("Linepos7:", LinePosLastLoop[7])
						ser.write(b'IR') #send gap, turn rigth
					else:
						ser.write(b'I') #send gap
				
			else:
				gapcounter = 0
				ser.write(str(linePos / 10).encode()) 

		if(grn_counter > 0):
			grn_counter = grn_counter - 1
		framesTotal = framesTotal + 1

		rawCapture.truncate(0)
		DEBUG()

		# save last 7 positions of the line
		LinePosLastLoop[7] = LinePosLastLoop[6]
		LinePosLastLoop[6] = LinePosLastLoop[5] 
		LinePosLastLoop[5] = LinePosLastLoop[4] 
		LinePosLastLoop[4] = LinePosLastLoop[3] 
		LinePosLastLoop[3] = LinePosLastLoop[2]
		LinePosLastLoop[2] = LinePosLastLoop[1]
		LinePosLastLoop[1] = LinePosLastLoop[0]
		LinePosLastLoop[0] = value


		LinePosLastLoop[0] = value
		for i in range(1, 8):
			LinePosLastLoop[i] = LinePosLastLoop[i - 1]

		key = cv2.waitKey(1) & 0xFF
		if key == ord("q"):

			print("Avg. FPS:", int(framesTotal / (time.time() - startTime - timeWaitet))) #sendet durchsch. Bilder pro Sekunde (FPS)
			camera.close()
			exit()