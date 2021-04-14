#!/usr/bin/env python
# coding=utf-8


# To do:
# Bei Lücke ein Stückchen in die richtige Richtung drehen (ein paar Werte, bevor weiß kam schauen, ob Linienpos rechts oder links war und dann ein Stück koriggieren)
# Grüne Punkte besser erkennen
# Dose umfahren und sich dabei nicht von anderen Linien irritieren lassen (neues ROI, ganz links am Kamerabild bzw. einfach alles rechts abschneiden)
# T Platte schaffen
# Silber erkennen verbessern
# Rescue Kit aufnehmen können
# Rescue Kit erkennen können (richtige Farbwerte herausfinden!!!)
# Rescue Kit am Anfag des Rescuebereichs ablegen
# Lebendes und totes Opfer unterscheiden
# Kugeln einzeln suchen und zur Ecke bringen
# Ausgang des Rescuebereichs finden 
# Ende des Parkours erkennen

from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import argparse
import time
import cv2
import serial
"""
RESOLUTION = (320, 192)
CUT = (50, 270, 120, 170) #eigentlich (50, 270, 120, 192)
CUT_GRN = (50, 270, 120, 192) #eigentlich (50, 270, 120, 192)
CUT_SILVER = (0, 100, 0, 192)  
CUT_RESCUEKIT = (50, 270, 120, 170)
"""
RESOLUTION = [320, 192]
CUT = [120, 170, 50, 270] #eigentlich (50, 270, 120, 192)
CUT_GRN = [120, 192, 80, 240] #eigentlich (50, 270, 120, 192)
CUT_SILVER = [0, 70, 50, 270]  
CUT_RESCUEKIT = [50, 270, 120, 170]

ser = serial.Serial('/dev/ttyAMA0', 9600, timeout = 0.5) #USB "Adresse" und Baudrate des Arduinos

while(not ser.is_open):
	print("Waiting for Serial...")
	time.sleep(0.1)
print("Opened:", ser.name, "aka Teensy 3.6") 

camera = PiCamera()
camera.resolution = RESOLUTION #Aufloesung, je niedriger desto schneller das Program
camera.rotation = 0 #habe die Kamera um 180 Grad gedreht eingebaut
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(320, 192))
rawCaptureCircles = PiRGBArray(camera, size=(320, 192))
time.sleep(0.5) #wartet kurz, damit Arduino und Kamera bereit sind
framesTotal = 0 #erstellt er, um bei Eingabe von q die durchsch. FPS anzeigen zu koennen
startTime = time.time() #erstellt er, um bei Eingabe von q die durchsch. FPS anzeigen zu koennen
lastLinePos = 0
LinePosLastLoop = [0, 0, 0, 0, 0, 0, 0, 0]
value = 0
gapcounter = 0
grn_list = []
grn_counter = 0
rescueCounter = 0
rescue = False
mindist = 300 
x = 0
y = 0
r = 0

##########FUNKTIONEN#############
def DEBUG():
	cv2.imshow("image_rgb", image_rgb) #gibt das aktuelle Bild aus
	cv2.imshow("image_hsv", image)
	cv2.imshow("cut", cut)
	#cv2.imshow("cut_green", cut_grn)
	#cv2.setMouseCallback("mouseRGB", mouseRGB)
	#cv2.imshow("mouseRGB", image)
	#cv2.imshow("cut_silber", cut_silver)
	#cv2.imshow("cut_rescuekit", cut_rescuekit)

def DEBUG_LastLinePos():
	print("LinePosLastLoop[0] = ", LinePosLastLoop[0])
	print("LinePosLastLoop[1] = ", LinePosLastLoop[1])
	print("LinePosLastLoop[2] = ", LinePosLastLoop[2])
	print("LinePosLastLoop[3] = ", LinePosLastLoop[3])
	print("LinePosLastLoop[4] = ", LinePosLastLoop[4])
	print("LinePosLastLoop[5] = ", LinePosLastLoop[5])
	print("LinePosLastLoop[6] = ", LinePosLastLoop[6])
	print("LinePosLastLoop[7] = ", LinePosLastLoop[7])

def mouseRGB(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN: #checks mouse left button down condition
        colorsB = image_rgb[y, x, 0]
        colorsG = image_rgb[y, x, 1]
        colorsR = image_rgb[y, x, 2]
        colors = image_rgb[y, x]
        print("Red: ", colorsR)
        print("Green: ", colorsG)
        print("Blue: ", colorsB)
        print("BRG Format: ", colors)
        print("Coordinates of pixel: X: ", x,"Y: ", y)

##############################################################################################

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):	
	image = frame.array #speichert das aktuelle Bild der Kamera in Variable ab
	image_rgb = image

	image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) # Konvertiert das Bild zum Christentum
	cv2.GaussianBlur(image, ((9, 9)), 2, 2)

	cut = image[120:170, 50:270] # Teil des frames fuer die Linienerkennung ausschneiden
	cut_grn = image[120:192, 80:240] # Teil des frames fuer die Gruenerkennung ausschneiden (etwas groeßer)
	cut_silver = image[0:70, 50:270]
	cut_rescuekit = image[50:270, 120:170]
	"""
	cut = image[CUT[0]:CUT[1]][CUT[2]:CUT[3]] # Teil des frames fuer die Linienerkennung ausschneiden
	cut_grn = image[CUT_GRN[0]:CUT_GRN[1]][CUT_GRN[2]:CUT_GRN[3]] # Teil des frames fuer die Gruenerkennung ausschneiden (etwas groeßer)
	cut_silver = image[CUT_SILVER[0]:CUT_SILVER[1]][CUT_SILVER[2]:CUT_SILVER[3]]
	cut_rescuekit = image[CUT_RESCUEKIT[0]:CUT_RESCUEKIT[1]][CUT_RESCUEKIT[2]:CUT_RESCUEKIT[3]]
	"""
	cv2.GaussianBlur(cut_silver, ((9, 9)), 2, 2) #den Bereich für die Silbererkennung noch mal extra verschwimmen lassen	

	line = cv2.inRange(cut, (0, 0, 0), (255, 255, 75)) # Kalibrierung schwarz eigentlich (0, 0, 0), (255, 255, 75))
	green = cv2.inRange(cut_grn, (55, 40, 40), (80, 255, 255)) # Kalibrierung gruen	eigentlich (55, 40, 40), (80, 255, 255)
	silber = cv2.inRange(cut_silver, (0, 0, 0), (255, 255, 75))
	rescuekit = cv2.inRange(cut_silver, (0, 0, 0), (255, 255, 75)) #richtige Werte eintragen

	contours_blk, hierarchy_blk = cv2.findContours(line.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	contours_grn, hierarchy_grn = cv2.findContours(green.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	contours_silver, hierarchy_silver = cv2.findContours(silber.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	contours_rescuekit, hierarchy_rescuekit = cv2.findContours(rescuekit.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	
	linePos = 0
	index = 0

	### Silbererkennung:
	if len(contours_silver) > 0: #falls die Kontur breiter als 0px ist / falls er Kontur findet
	   x_silber, y_silber, w_silber, h_silber = cv2.boundingRect(contours_silver[0]) # erstellt Rechteck um die Konturen 
	   cv2.rectangle(image, (x_silber, y_silber), (x_silber + w_silber, y_silber + h_silber), (255, 0, 0), 3) #malt dieses Rechteck auch ins Bild
	   #print("kein silber erkannt")
	   if rescueCounter > 2:
	   	rescueCounter = rescueCounter - 3
	if len(contours_silver) == 0 :
		rescueCounter = rescueCounter + 1
		if rescueCounter > 10: #hat 10 mal in Folge kein schwarz gesehen -> da ist (wahrscheinlich)	der Rescue Bereich
			print("silber entgueltig erkannt")
			ser.write(b'Rescue') #sendet an den Teensy, dass er silber erkannt hat
			read_serial = ser.readline().decode('ascii') #schaut, ob Daten (in diesem Fall trigger fuer Rescuebereich) empfangen wurden
			if read_serial == '8\r\n': #da ist wirklich der Rescuebereich
				time.sleep(0.1)
				cv2.destroyAllWindows()
				break
			else:
				print("Der Teensy hat gesagt, dass es doch nicht der Rescue ist")
				ser.write(str(0/10).encode())
			rescueCounter = 0
	if(len(contours_blk) > 0):
		nearest = 1000
		for i in range(len(contours_blk)):
			b = cv2.boundingRect(contours_blk[i])
			x, y, w, h = b
			a = int(abs(x + w / 2 - 160 - lastLinePos))
			cv2.rectangle(image, (x, y + 80), (x + w, y + 80 + h), (0, 100, 0), 3)
			if(a < nearest):
				nearest = a
				index = i
		b = cv2.boundingRect(contours_blk[index])
		x, y, w, h = b
		#print(w)
		if(w > 319): #falls sehr viel schwarz zu sehen ist, sendet er das an den Arduino
			ser.write(b'S')
			#print("Skipped")
			#time.sleep(0.15)
		linePos = int(x + w / 2 - 160)
		cv2.rectangle(image, (x, y + 80), (x + w, y + 80 + h), (0, 0, 255), 3)
		cv2.putText(image, str(linePos),(linePos + 160, 50), cv2.FONT_HERSHEY_SCRIPT_SIMPLEX, 1, (255, 0, 0), 2)
		cv2.line(image, (linePos + 160, 80), (linePos + 160, 160), (255, 0, 0),2)
		cv2.line(image, (0, 110), (319, 110), (255, 0, 0), 2)
		lastLinePos = linePos
	contours_right = False
	contours_left = False	
	if(len(contours_grn) > 0):
		if(grn_counter <= 0):
			grn_counter = 10
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
				if(d): #hat doppel Gruen erkannt
					ser.write(b'D') 
					print("deadend") 
					print("Send: D")
					time.sleep(1)
				elif(s >= 6):
					#ser.write(b'S')
					#time.sleep(0.2)
					print("Send: S")
				else:
					if(left > right):
						ser.write(b'L')
						print("Send: L")
						time.sleep(0.5)
					elif(right > left):
						ser.write(b'R')
						print("Send: R")
						time.sleep(0.5)
				grn_counter = 0
				grn_list.clear()
				#print("List cleared!")
				# for c in grn_list:
				# 	print(c)
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
				cv2.rectangle(image, (x, y + 50), (x + w, y + 50 + h), (0, 255, 0), 3)
				a = x + w/2 - 160
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
			cv2.rectangle(image, (x, y + 50), (x + w, y + 50 + h), (0, 0, 255), 3)
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
			cv2.rectangle(image, (x, y + 50), (x + w, y + 50 + h), (0, 255, 0), 3)
			a = x + w/2 - 160
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
			cv2.rectangle(image, (x, y + 50), (x + w, y + 50 + h), (0, 255, 0), 3)
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

		if value == 0: #value == 0 -> Roboter befindet sich in einer Luecke, weil er keine Linie finden kann
			print(gapcounter)
			gapcounter = gapcounter + 1
			"""
			if gapcounter >= 5:
				#Luecke erkannt
				gapcounter = 0
				print("Luecke erkannt")
				if LinePosLastLoop[7] > 10: #vor 6 Loop Durchlaeufen war str(linePos).encode() > 0, weshalb sich der Roboter jetzt ein Stueck nach rechts drehen muss, um die Luecke besser zu ueberfahren
					print("gapR -> nach rechts drehen...")
					time.sleep(5)
					#ser.write(b'gapR')
					time.sleep(0.05)
				elif LinePosLastLoop[7] < -10: #vor 6 Loop Durchlaeufen war str(linePos).encode() < 0, weshalb sich der Roboter jetzt ein Stueck nach links drehen muss, um die Luecke besser zu ueberfahren
					print("gapL -> nach links drehen...")
					time.sleep(5)
					#ser.write(b'gapL')
					time.sleep(0.05)
				elif LinePosLastLoop[7] == 0:
					print("LinePosLastLoop[7] ist leider 0")
				else: #fahre ein Stückchen gerade aus
					pass
			"""
		else:
			gapcounter = 0
			ser.write(str(linePos / 10).encode()) 

	if(grn_counter > 0):
		grn_counter = grn_counter - 1
	framesTotal = framesTotal + 1
	rawCapture.truncate(0)
	DEBUG()
	# wird gebraucht, um bei Luecke (-> gapcounter >= Schwellwert) ein Stueck nach rechts oder links zu korrigieren
	LinePosLastLoop[7] = LinePosLastLoop[6]
	LinePosLastLoop[6] = LinePosLastLoop[5] 
	LinePosLastLoop[5] = LinePosLastLoop[4] 
	LinePosLastLoop[4] = LinePosLastLoop[3] 
	LinePosLastLoop[3] = LinePosLastLoop[2] #speichert den vor-vor-vor-vorletzten str(linePos).encode() in Array
	LinePosLastLoop[2] = LinePosLastLoop[1] #speichert den vor-vor-vorletzten str(linePos).encode() in Array
	LinePosLastLoop[1] = LinePosLastLoop[0] #speichert den vor-vorletzten str(linePos).encode() in Array
	LinePosLastLoop[0] = value #speichert den vorletzten str(linePos).encode() in Array


	key = cv2.waitKey(1) & 0xFF
	if key == ord("q"):
		print("Avg. FPS:", int(framesTotal / (time.time()-startTime))) #sendet durchsch. Bilder pro Sekunde (FPS)
		break


circlesCounter = 0
ResolutionRescue = (320, 192)
camera = PiCamera()
camera.resolution = ResolutionRescue #Aufloesung, je niedriger desto schneller das Program
camera.rotation = 0
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=ResolutionRescue)
rawCaptureCircles = PiRGBArray(camera, size=ResolutionRescue)
time.sleep(0.5) #wartet kurz, damit Teensy und Kamera bereit sind
print("Rescue program started")
framesTotalRescue = 0 #erstellt er, um bei Eingabe von q die durchsch. FPS anzeigen zu koennen
startTimeRescue = time.time() #erstellt er, um bei Eingabe von q die durchsch. FPS anzeigen zu koennen
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	image = frame.array
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	# detect circles in the image
	#circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 2.5, 300)
	circles = cv2.HoughCircles(image, cv2.HOUGH_GRADIENT, dp = 1, minDist = 60, param1 = 34, param2 = 24, minRadius = 2, maxRadius = 300)
	# ensure at least some circles were found
	if circles is not None:
		circlesCounter = circlesCounter + 1
		# convert the (x, y) coordinates and radius of the circles to integers
		circles = np.round(circles[0, :]).astype("int")
		# loop over the (x, y) coordinates and radius of the circles
		for (x, y, r) in circles:
			#print(y) # y = naehe bzw y Achse
			# draw the circle in the output image, then draw a rectangle
			cv2.circle(image, (x, y), r, (255, 255, 0), 4)
			cv2.rectangle(image, (x - 5, y - 5), (x + 5, y + 5), (0, 0, 255), -1)
			ballPosition = int((x - 160) / 10)
			if ballPosition > 1:
				print("send: R")
				#ser.write(b'R')
			if ballPosition < -1:
				print("send: L")
				#ser.write(b'L')
			if ballPosition > -1 and ballPosition < 1:
				print("Ist mittig")
				if(y >= 95 and not y == 0 and y <= 100):
					#ser.write(b'P') #sende Befehl fuer Drehung
					print("perfekt, drehe dich")
				if(y <= 95 and not y == 0):
					#ser.write(b'V') #sende Befehl fuer naeher ran fahren
					print("vor fahren")
				if(y >= 101 and not y == 0):
					#ser.write(b'Z') #sende Befehl fuer naeher ran fahren
					print("zurueck fahren")
			#print(y)
			#ser.write(str((x - 160) / 10).encode()) # eigentlich : ser.write(str((x-160)/10).encode())	
	rawCapture.truncate(0)
	cv2.imshow("Kugel output", image)
	key = cv2.waitKey(1) & 0xFF
	framesTotalRescue = framesTotalRescue + 1
	if key == ord("q"):
		print("Avg. FPS:", int(framesTotalRescue / (time.time() - startTimeRescue))) #sendet durchsch. Bilder pro Sekunde (FPS)
		break #beendet Program bzw. bricht aus Endlosschleife aus
print("Program beendet")
